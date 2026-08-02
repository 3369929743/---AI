#include "Ball_Contral.h"
#include "main.h"

#define BALL_VELOCITY_FILTER_TAU_MS 60.0f
#define BALL_FRAME_DT_MIN_MS 2U
#define BALL_FRAME_DT_MAX_MS 200U
#define BALL_INTEGRAL_ERROR_LIMIT 28.0f
#define BALL_INTEGRAL_SPEED_LIMIT 120.0f
#define BALL_PID_OUTPUT_POLARITY (-1.0f)
/*
 * 电机命令死区，单位是脉冲而不是视觉像素。
 * 新旧管道目标相差不足 3 脉冲时，不向电机发送新命令。
 */
#define BALL_MOTOR_COMMAND_DEADBAND_PULSE 3

static PID_val BallContral_Calculate(BallContral_t *BallContral,
                                     PID_val Actual)
{
    PID_t *PID = &BallContral->PID_StepMotor;
    uint32_t Now = HAL_GetTick();
    uint32_t Dt_ms = Now - BallContral->Last_Frame_Tick;
    PID_val Dt_s = 0.0f;
    PID_val Raw_Error;

    if(!BallContral->Has_Ball_History){
        BallContral->Ball_Position_Pre = Actual;
        BallContral->Ball_Velocity = 0.0f;
        BallContral->Has_Ball_History = 1U;
    }
    else if(Dt_ms >= BALL_FRAME_DT_MIN_MS && Dt_ms <= BALL_FRAME_DT_MAX_MS){
        PID_val Alpha = (PID_val)Dt_ms
                      / (BALL_VELOCITY_FILTER_TAU_MS + (PID_val)Dt_ms);
        PID_val Raw_Velocity = (Actual - BallContral->Ball_Position_Pre)
                             * 1000.0f / (PID_val)Dt_ms;

        BallContral->Ball_Velocity += Alpha
                                    * (Raw_Velocity
                                       - BallContral->Ball_Velocity);
        Dt_s = (PID_val)Dt_ms * 0.001f;
    }
    else{
        BallContral->Ball_Velocity = 0.0f;
    }

    BallContral->Ball_Position_Pre = Actual;
    BallContral->Last_Frame_Tick = Now;

    PID->Actual = Actual;
    PID->Pre_Error = PID->Cur_Error;
    Raw_Error = PID->Target - PID->Actual;
    PID->Cur_Error = BallContral_Apply_Position_Deadband(
        Raw_Error,
        BallContral->Position_Deadband);
    PID->Error_Rate_Filter = -BallContral->Ball_Velocity;

    if(Dt_s > 0.0f){
        PID->ErrorInt += PID->Cur_Error * Dt_s;
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

static void BallContral_Apply_Output(BallContral_t *BallContral,
                                     PID_val Output)
{
    int32_t New_Target_Pulse;
    int32_t Delta_Pulse;

    if(Output >= 0.0f){
        New_Target_Pulse = (int32_t)(Output + 0.5f);
    }
    else{
        New_Target_Pulse = (int32_t)(Output - 0.5f);
    }

    /* PID 输出表示管道的绝对目标偏移，只发送相对上次目标的差值。 */
    Delta_Pulse = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Delta_Pulse >= BALL_MOTOR_COMMAND_DEADBAND_PULSE
       || Delta_Pulse <= -BALL_MOTOR_COMMAND_DEADBAND_PULSE){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Delta_Pulse);
        BallContral->Pipe_Target_Pulse = New_Target_Pulse;
    }
}

void BallContral_Init(BallContral_t *BallContral,
                      Serial_t *Serial_Emm,
                      PID_Confg_t *PID_Confg)
{
    Emm_Init(&BallContral->Emm_StepMotor, Serial_Emm);
    Emm_SetSpeed(&BallContral->Emm_StepMotor, 500U);
    PID_Init(&BallContral->PID_StepMotor, PID_Confg);

    BallContral->is_Enable = 0U;
    BallContral->Pipe_Target_Pulse = 0;
    BallContral->Position_Deadband = 0.0f;
    BallContral->Ball_Position_Pre = 0.0f;
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Last_Frame_Tick = 0U;
    BallContral->Has_Ball_History = 0U;

    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
}

void Ball_Contral_Emm_Quick_Init(BallContral_t *BallContral)
{
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
}

void Ball_Contral_Pop_Run(BallContral_t *BallContral, int32_t Pulse)
{
    if(BallContral->is_Enable) return;
    Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Pulse);
}

void BallContral_Set_Target(BallContral_t *BallContral,
                            PID_val Target,
                            PID_val Deadband)
{
    PID_Set_Target(&BallContral->PID_StepMotor, Target);
    BallContral->Position_Deadband = (Deadband >= 0.0f)
                                   ? Deadband
                                   : -Deadband;
}

void BallContral_Clear_Integral(BallContral_t *BallContral)
{
    BallContral->PID_StepMotor.ErrorInt = 0.0f;
}

void BallContral_Run(BallContral_t *BallContral, PID_val Actual)
{
    PID_val Output;

    if(!BallContral->is_Enable) return;

    Output = BallContral_Calculate(BallContral, Actual);
    BallContral_Apply_Output(BallContral, Output);
}

uint8_t BallContral_Get_is_Enable(BallContral_t *BallContral)
{
    return BallContral->is_Enable;
}

void BallContral_Start(BallContral_t *BallContral)
{
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0U;
    BallContral->is_Enable = 1U;
}

void BallContral_Stop(BallContral_t *BallContral)
{
    if(BallContral->is_Enable && BallContral->Pipe_Target_Pulse != 0){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor,
                          -BallContral->Pipe_Target_Pulse);
        BallContral->Pipe_Target_Pulse = 0;
    }

    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0U;
    BallContral->is_Enable = 0U;
}
