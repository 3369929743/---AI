#include "Ball_Contral.h"
#include "main.h"

/* Position hold: filtered PID plus basic actuator limits. */
#define BALL_HOLD_VELOCITY_FILTER_TAU_MS 60.0f
#define BALL_FRAME_DT_MIN_MS 2U
#define BALL_FRAME_DT_MAX_MS 200U
#define BALL_PID_OUTPUT_POLARITY (-1.0f)
#define BALL_HOLD_OUTPUT_STEP_LIMIT 80
#define BALL_POSITION_MOTOR_SPEED 500U
#define BALL_POSITION_MOTOR_ACC 0U
#define BALL_HOLD_MOTOR_SPEED 1100U
#define BALL_HOLD_MOTOR_ACC 240U

/*
 * 保持模式静止锁定参数，误差单位为视觉像素，速度单位为像素/秒。
 * 进入门限小、释放门限大，形成滞环，避免在边界反复锁定和解锁。
 */
#define BALL_HOLD_LOCK_ERROR            3.0f
#define BALL_HOLD_LOCK_SPEED            25.0f
#define BALL_HOLD_LOCK_FRAMES           4U
#define BALL_HOLD_RELEASE_ERROR         6.0f
#define BALL_HOLD_RELEASE_SPEED         50.0f
#define BALL_HOLD_RELEASE_FRAMES        2U

static PID_val BallContral_Hold_Abs(PID_val Value)
{
    return (Value < 0.0f) ? -Value : Value;
}

static void BallContral_Hold_Reset_Lock(BallContral_t *BallContral)
{
    BallContral->Hold_Is_Locked = 0;
    BallContral->Hold_Lock_Frames = 0;
    BallContral->Hold_Release_Frames = 0;
}

static void BallContral_Hold_Reset_State(BallContral_t *BallContral)
{
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
    BallContral_Hold_Reset_Lock(BallContral);
}

void BallContral_Position_Mode_Init(BallContral_t *BallContral)
{
    Emm_SetSpeed(&BallContral->Emm_StepMotor, BALL_POSITION_MOTOR_SPEED);
    Emm_SetAcc(&BallContral->Emm_StepMotor, BALL_POSITION_MOTOR_ACC);
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
    BallContral_Set_Output_Step_Limit(BallContral, 0);
    BallContral->Feedback_Output = 0.0f;
    BallContral_Set_Feedforward_Output(BallContral, 0.0f);
    BallContral_Hold_Reset_State(BallContral);
}

void BallContral_Hold_Mode_Init(BallContral_t *BallContral)
{
    Emm_SetSpeed(&BallContral->Emm_StepMotor, BALL_HOLD_MOTOR_SPEED);
    Emm_SetAcc(&BallContral->Emm_StepMotor, BALL_HOLD_MOTOR_ACC);
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
    BallContral_Set_Output_Step_Limit(BallContral,
                                     BALL_HOLD_OUTPUT_STEP_LIMIT);
    BallContral->Feedback_Output = 0.0f;
    BallContral_Set_Feedforward_Output(BallContral, 0.0f);
    BallContral_Hold_Reset_State(BallContral);
    BallContral->Hold_Target = 0.0f;
    PID_Set_Target(&BallContral->PID_StepMotor, BallContral->Hold_Target);
}

static PID_val BallContral_Calculate_Filtered_PID(BallContral_t *BallContral,
                                                   PID_val Actual,
                                                   uint8_t Allow_Integral)
{
    PID_t *PID = &BallContral->PID_StepMotor;
    uint32_t Now = HAL_GetTick();
    uint32_t Dt_ms = Now - BallContral->Last_Frame_Tick;
    PID_val Dt_s = 0.0f;
    PID_val Unsaturated_Output;
    PID_val Integral_Output_Change;

    if(!BallContral->Has_Ball_History){
        BallContral->Ball_Position_Pre = Actual;
        BallContral->Ball_Velocity = 0.0f;
        BallContral->Has_Ball_History = 1;
    }
    else if(Dt_ms >= BALL_FRAME_DT_MIN_MS && Dt_ms <= BALL_FRAME_DT_MAX_MS){
        PID_val Alpha = (PID_val)Dt_ms
                      / (BALL_HOLD_VELOCITY_FILTER_TAU_MS + (PID_val)Dt_ms);
        PID_val Raw_Velocity = (Actual - BallContral->Ball_Position_Pre)
                             * 1000.0f / (PID_val)Dt_ms;

        BallContral->Ball_Velocity += Alpha
                                    * (Raw_Velocity - BallContral->Ball_Velocity);
        Dt_s = (PID_val)Dt_ms * 0.001f;
    }
    else{
        BallContral->Ball_Velocity = 0.0f;
    }

    BallContral->Ball_Position_Pre = Actual;
    BallContral->Last_Frame_Tick = Now;

    PID->Target = BallContral->Hold_Target;
    PID->Actual = Actual;
    PID->Pre_Error = PID->Cur_Error;
    PID->Cur_Error = PID->Target - PID->Actual;
    PID->Error_Rate_Filter = -BallContral->Ball_Velocity;

    Unsaturated_Output = BALL_PID_OUTPUT_POLARITY
                       * (PID->Kp * PID->Cur_Error
                        + PID->Ki * PID->ErrorInt
                        - PID->Kd * BallContral->Ball_Velocity);

    /*
     * Conditional integration prevents windup at the output limits. Integrate
     * while unsaturated, or when the new integral drives a saturated command
     * back toward the valid range.
     */
    if(Allow_Integral && Dt_s > 0.0f && PID->Ki != 0.0f){
        Integral_Output_Change = BALL_PID_OUTPUT_POLARITY
                               * PID->Ki * PID->Cur_Error * Dt_s;
        if((Unsaturated_Output < PID->OutMax
            && Unsaturated_Output > PID->OutMin)
           || (Unsaturated_Output >= PID->OutMax
               && Integral_Output_Change < 0.0f)
           || (Unsaturated_Output <= PID->OutMin
               && Integral_Output_Change > 0.0f)){
            PID->ErrorInt += PID->Cur_Error * Dt_s;
        }
    }

    if(PID->ErrorInt > PID->IntMax) PID->ErrorInt = PID->IntMax;
    else if(PID->ErrorInt < PID->IntMin) PID->ErrorInt = PID->IntMin;

    PID->Output = BALL_PID_OUTPUT_POLARITY
                * (PID->Kp * PID->Cur_Error
                 + PID->Ki * PID->ErrorInt
                 - PID->Kd * BallContral->Ball_Velocity);

    if(PID->Output > PID->OutMax) PID->Output = PID->OutMax;
    else if(PID->Output < PID->OutMin) PID->Output = PID->OutMin;

    return PID->Output;
}

void BallContral_Run_Position_Hold(BallContral_t *BallContral, PID_val Target)
{
    PID_val Output;
    PID_val Error;
    PID_val Speed;
    uint8_t Was_Locked;

    if(!BallContral->is_Enable) return;

    Was_Locked = BallContral->Hold_Is_Locked;
    Output = BallContral_Calculate_Filtered_PID(BallContral,
                                                Target,
                                                !Was_Locked);
    Error = BallContral_Hold_Abs(BallContral->PID_StepMotor.Cur_Error);
    Speed = BallContral_Hold_Abs(BallContral->Ball_Velocity);

    if(Was_Locked){
        /*
         * 锁定期间继续观察球的位置和速度，但冻结积分和当前管道目标，
         * 因此视觉小抖动不会再产生电机命令。
         */
        if(Error > BALL_HOLD_RELEASE_ERROR
           || Speed > BALL_HOLD_RELEASE_SPEED){
            if(BallContral->Hold_Release_Frames < 0xFFU){
                BallContral->Hold_Release_Frames++;
            }
        }
        else{
            BallContral->Hold_Release_Frames = 0;
        }

        if(BallContral->Hold_Release_Frames < BALL_HOLD_RELEASE_FRAMES){
            return;
        }

        /* 连续偏离后恢复 PID，第一帧仍不积分，避免解锁瞬间积分突跳。 */
        BallContral->Hold_Is_Locked = 0;
        BallContral->Hold_Lock_Frames = 0;
        BallContral->Hold_Release_Frames = 0;
    }
    else{
        if(Error <= BALL_HOLD_LOCK_ERROR
           && Speed <= BALL_HOLD_LOCK_SPEED){
            if(BallContral->Hold_Lock_Frames < 0xFFU){
                BallContral->Hold_Lock_Frames++;
            }
        }
        else{
            BallContral->Hold_Lock_Frames = 0;
        }

        if(BallContral->Hold_Lock_Frames >= BALL_HOLD_LOCK_FRAMES){
            BallContral->Hold_Is_Locked = 1;
            BallContral->Hold_Release_Frames = 0;
        }
    }

    /* 锁定前保留最后一拍 PID 输出，作为锁定期间固定的管道角度。 */
    BallContral_Set_Feedback_Output(BallContral, Output);
}
