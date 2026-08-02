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

static PID_val BallContral_Calculate(BallContral_t *BallContral, PID_val Actual)
{
    PID_t *PID = &BallContral->PID_StepMotor;
    uint32_t Now = HAL_GetTick();
    uint32_t Dt_ms = Now - BallContral->Last_Frame_Tick;
    PID_val Dt_s = 0.0f;

    if(!BallContral->Has_Ball_History){
        BallContral->Ball_Position_Pre = Actual;
        BallContral->Ball_Velocity = 0.0f;
        BallContral->Has_Ball_History = 1;
    }
    else if(Dt_ms >= BALL_FRAME_DT_MIN_MS && Dt_ms <= BALL_FRAME_DT_MAX_MS){
        PID_val Alpha = (PID_val)Dt_ms / (BALL_VELOCITY_FILTER_TAU_MS + (PID_val)Dt_ms);
        PID_val Raw_Velocity = (Actual - BallContral->Ball_Position_Pre) * 1000.0f / (PID_val)Dt_ms;

        BallContral->Ball_Velocity += Alpha * (Raw_Velocity - BallContral->Ball_Velocity);
        Dt_s = (PID_val)Dt_ms * 0.001f;
    }
    else{
        BallContral->Ball_Velocity = 0.0f;
    }

    BallContral->Ball_Position_Pre = Actual;
    BallContral->Last_Frame_Tick = Now;

    PID->Actual = Actual;
    PID->Pre_Error = PID->Cur_Error;
    PID->Cur_Error = PID->Target - PID->Actual;
    PID->Error_Rate_Filter = -BallContral->Ball_Velocity;

    if(Dt_s > 0.0f
       && ((PID->Cur_Error >= 0.0f ? PID->Cur_Error : -PID->Cur_Error) <= BALL_INTEGRAL_ERROR_LIMIT)
       && ((BallContral->Ball_Velocity >= 0.0f ? BallContral->Ball_Velocity : -BallContral->Ball_Velocity) <= BALL_INTEGRAL_SPEED_LIMIT)){
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

/**
  * @brief  初始化球体控制模块
  * @param  BallContral: 球体控制结构体指针
  * @param  Serial_K230: K230串口通信结构体指针
  * @param  Serial_Emm: Emm步进电机串口通信结构体指针
  * @param  PID_Confg: PID配置参数结构体指针
  * @retval 无
  */
void BallContral_Init(BallContral_t *BallContral, Serial_t *Serial_K230, Serial_t *Serial_Emm, PID_Confg_t *PID_Confg)
{
    /* 保存串口通信句柄 */
    BallContral->Serial_K230 = Serial_K230;
    BallContral->Serial_Emm = Serial_Emm;

    /* 初始化Emm步进电机和PID控制器 */
    Emm_Init(&BallContral->Emm_StepMotor, Serial_Emm);
    Emm_SetSpeed(&BallContral->Emm_StepMotor, 500);
    PID_Init(&BallContral->PID_StepMotor, PID_Confg);
    BallContral->is_Enable = 0;
    BallContral->Pipe_Target_Pulse = 0;
    BallContral->Ball_Position_Pre = 0.0f;
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Last_Frame_Tick = 0;
    BallContral->Has_Ball_History = 0;

    /* 初始化Emm位置快速控制模式 */
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
}

void Ball_Contral_Emm_Quick_Init(BallContral_t *BallContral){
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
}

void Ball_Contral_Pop_Run(BallContral_t *BallContral, int32_t Pulse){
    if(BallContral->is_Enable) return;
    Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Pulse);
}

void BallContral_Set_Target(BallContral_t *BallContral, PID_val Target)
{
    PID_Set_Target(&BallContral->PID_StepMotor, Target);
}

void BallContral_Clear_Integral(BallContral_t *BallContral)
{
    BallContral->PID_StepMotor.ErrorInt = 0.0f;
}

void BallContral_Run(BallContral_t *BallContral, PID_val Target){
    PID_val Emm;
    int32_t New_Target_Pulse;
    int32_t Delta_Pulse;

    if(!BallContral->is_Enable) return;

    Emm = BallContral_Calculate(BallContral, Target);

    /* PID output is the desired pipe offset, not a repeatedly accumulated step. */
    if(Emm >= 0){
        New_Target_Pulse = (int32_t)(Emm + 0.5f);
    }
    else{
        New_Target_Pulse = (int32_t)(Emm - 0.5f);
    }

    Delta_Pulse = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Delta_Pulse >= BALL_MOTOR_COMMAND_DEADBAND_PULSE ||
       Delta_Pulse <= -BALL_MOTOR_COMMAND_DEADBAND_PULSE){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Delta_Pulse);
        BallContral->Pipe_Target_Pulse = New_Target_Pulse;
    }
}

uint8_t BallContral_Get_is_Enable(BallContral_t *BallContral){
    return BallContral->is_Enable;
}

void BallContral_Start(BallContral_t *BallContral){
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
    BallContral->is_Enable = 1;
}

void BallContral_Stop(BallContral_t *BallContral){
    if(BallContral->is_Enable && BallContral->Pipe_Target_Pulse != 0){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, -BallContral->Pipe_Target_Pulse);
        BallContral->Pipe_Target_Pulse = 0;
    }
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
    BallContral->is_Enable = 0;
}
