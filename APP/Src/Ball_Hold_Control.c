#include "Ball_Contral.h"
#include "main.h"

/* Position hold: filtered PID plus basic actuator limits. */
#define BALL_HOLD_VELOCITY_FILTER_TAU_MS 60.0f
#define BALL_FRAME_DT_MIN_MS 2U
#define BALL_FRAME_DT_MAX_MS 200U
#define BALL_PID_OUTPUT_POLARITY (-1.0f)
#define BALL_HOLD_COMMAND_DEADBAND_PULSE 3
#define BALL_HOLD_OUTPUT_STEP_LIMIT 80
#define BALL_POSITION_MOTOR_SPEED 500U
#define BALL_POSITION_MOTOR_ACC 0U
#define BALL_HOLD_MOTOR_SPEED 1100U
#define BALL_HOLD_MOTOR_ACC 240U

static void BallContral_Hold_Reset_State(BallContral_t *BallContral)
{
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
}

void BallContral_Position_Mode_Init(BallContral_t *BallContral)
{
    Emm_SetSpeed(&BallContral->Emm_StepMotor, BALL_POSITION_MOTOR_SPEED);
    Emm_SetAcc(&BallContral->Emm_StepMotor, BALL_POSITION_MOTOR_ACC);
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
    BallContral_Hold_Reset_State(BallContral);
}

void BallContral_Hold_Mode_Init(BallContral_t *BallContral)
{
    Emm_SetSpeed(&BallContral->Emm_StepMotor, BALL_HOLD_MOTOR_SPEED);
    Emm_SetAcc(&BallContral->Emm_StepMotor, BALL_HOLD_MOTOR_ACC);
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
    BallContral_Hold_Reset_State(BallContral);
    BallContral->Hold_Target = 0.0f;
    PID_Set_Target(&BallContral->PID_StepMotor, BallContral->Hold_Target);
}

static PID_val BallContral_Calculate_Filtered_PID(BallContral_t *BallContral,
                                                   PID_val Actual)
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
    if(Dt_s > 0.0f && PID->Ki != 0.0f){
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
    int32_t New_Target_Pulse;
    int32_t Output_Step;
    int32_t Delta_Pulse;

    if(!BallContral->is_Enable) return;

    Output = BallContral_Calculate_Filtered_PID(BallContral, Target);
    if(Output >= 0.0f){
        New_Target_Pulse = (int32_t)(Output + 0.5f);
    }
    else{
        New_Target_Pulse = (int32_t)(Output - 0.5f);
    }

    Output_Step = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Output_Step > BALL_HOLD_OUTPUT_STEP_LIMIT){
        New_Target_Pulse = BallContral->Pipe_Target_Pulse
                         + BALL_HOLD_OUTPUT_STEP_LIMIT;
    }
    else if(Output_Step < -BALL_HOLD_OUTPUT_STEP_LIMIT){
        New_Target_Pulse = BallContral->Pipe_Target_Pulse
                         - BALL_HOLD_OUTPUT_STEP_LIMIT;
    }

    Delta_Pulse = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Delta_Pulse >= BALL_HOLD_COMMAND_DEADBAND_PULSE
       || Delta_Pulse <= -BALL_HOLD_COMMAND_DEADBAND_PULSE){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Delta_Pulse);
        BallContral->Pipe_Target_Pulse = New_Target_Pulse;
    }
}
