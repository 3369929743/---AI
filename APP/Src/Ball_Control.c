#include "Ball_Contral.h"
#include "main.h"

#define BALL_VELOCITY_FILTER_TAU_MS 60.0f
#define BALL_FRAME_DT_MIN_MS 2U
#define BALL_FRAME_DT_MAX_MS 200U
/*
 * 5 cm 位置模式的积分作用范围，单位是视觉像素。
 * 需要覆盖完整的 138 像素行程，否则球在误差大于 28 像素处卡住时，
 * 积分不会继续增加管道坡度，表现为还没到目标就停住。
 */
#define BALL_POSITION_INTEGRAL_ERROR_LIMIT 160.0f
#define BALL_INTEGRAL_SPEED_LIMIT 120.0f
#define BALL_PID_OUTPUT_POLARITY (-1.0f)
/*
 * 电机命令死区，单位是脉冲而不是视觉像素。
 * 新旧管道目标相差不足 3 脉冲时，不向电机发送新命令。
 */
#define BALL_MOTOR_COMMAND_DEADBAND_PULSE 3
/*
 * 保持 PID 给出普通管道目标；启停前馈给出“触发前基准+加速度偏移”。
 * Position_Blend 随钢球偏差由 1 降到 0，在绝对位置目标之间无扰切换，
 * 而不是把两个绝对位置命令直接相加。
 */
static void BallContral_Apply_Combined_Output(BallContral_t *BallContral)
{
    PID_val Combined_Output;
    int32_t New_Target_Pulse;
    int32_t Output_Step;
    int32_t Delta_Pulse;

    if(!BallContral->is_Enable) return;

    if(BallContral->Feedforward_Target_Active){
        PID_val Blend = BallContral->Feedforward_Blend;
        PID_val Feedforward_Target = BallContral->Feedforward_Baseline
                                   + BallContral->Feedforward_Output;

        if(Blend > 1.0f) Blend = 1.0f;
        else if(Blend < 0.0f) Blend = 0.0f;
        Combined_Output = BallContral->Feedback_Output
                        + Blend * (Feedforward_Target
                                   - BallContral->Feedback_Output);
    }
    else{
        Combined_Output = BallContral->Feedback_Output
                        + BallContral->Feedforward_Output;
    }
    if(Combined_Output > BallContral->PID_StepMotor.OutMax){
        Combined_Output = BallContral->PID_StepMotor.OutMax;
    }
    else if(Combined_Output < BallContral->PID_StepMotor.OutMin){
        Combined_Output = BallContral->PID_StepMotor.OutMin;
    }

    /* 超时折返时减小目标脉冲幅值，不改变输出方向。 */
    if(BallContral->Output_Pulse_Reduction > 0.0f){
        if(Combined_Output > BallContral->Output_Pulse_Reduction){
            Combined_Output -= BallContral->Output_Pulse_Reduction;
        }
        else if(Combined_Output < -BallContral->Output_Pulse_Reduction){
            Combined_Output += BallContral->Output_Pulse_Reduction;
        }
        else{
            Combined_Output = 0.0f;
        }
    }

    if(Combined_Output >= 0.0f){
        New_Target_Pulse = (int32_t)(Combined_Output + 0.5f);
    }
    else{
        New_Target_Pulse = (int32_t)(Combined_Output - 0.5f);
    }

    Output_Step = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(BallContral->Output_Step_Limit > 0){
        if(Output_Step > BallContral->Output_Step_Limit){
            New_Target_Pulse = BallContral->Pipe_Target_Pulse
                             + BallContral->Output_Step_Limit;
        }
        else if(Output_Step < -BallContral->Output_Step_Limit){
            New_Target_Pulse = BallContral->Pipe_Target_Pulse
                             - BallContral->Output_Step_Limit;
        }
    }

    Delta_Pulse = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Delta_Pulse >= BALL_MOTOR_COMMAND_DEADBAND_PULSE
       || Delta_Pulse <= -BALL_MOTOR_COMMAND_DEADBAND_PULSE){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Delta_Pulse);
        BallContral->Pipe_Target_Pulse = New_Target_Pulse;
    }
}

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
       && ((PID->Cur_Error >= 0.0f ? PID->Cur_Error : -PID->Cur_Error) <= BALL_POSITION_INTEGRAL_ERROR_LIMIT)
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
    BallContral->Feedback_Output = 0.0f;
    BallContral->Feedforward_Output = 0.0f;
    BallContral->Feedforward_Baseline = 0.0f;
    BallContral->Feedforward_Blend = 0.0f;
    BallContral->Feedforward_Target_Active = 0U;
    BallContral->Output_Step_Limit = 0;
    BallContral->Output_Pulse_Reduction = 0.0f;
    BallContral->Ball_Position_Pre = 0.0f;
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Last_Frame_Tick = 0;
    BallContral->Has_Ball_History = 0;
    BallContral->Hold_Integral_Frozen = 0;
    BallContral->Hold_Is_Locked = 0;
    BallContral->Hold_Lock_Frames = 0;
    BallContral->Hold_Release_Frames = 0;

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

void BallContral_Set_Output_Step_Limit(BallContral_t *BallContral,
                                      int32_t StepLimit)
{
    BallContral->Output_Step_Limit = (StepLimit > 0) ? StepLimit : 0;
}

void BallContral_Set_Output_Pulse_Reduction(BallContral_t *BallContral,
                                           PID_val Reduction)
{
    BallContral->Output_Pulse_Reduction =
        (Reduction > 0.0f) ? Reduction : 0.0f;
}

void BallContral_Set_Feedback_Output(BallContral_t *BallContral,
                                    PID_val Output)
{
    BallContral->Feedback_Output = Output;
    BallContral_Apply_Combined_Output(BallContral);
}

void BallContral_Set_Feedforward_Output(BallContral_t *BallContral,
                                       PID_val Output)
{
    BallContral->Feedforward_Target_Active = 0U;
    BallContral->Feedforward_Baseline = 0.0f;
    BallContral->Feedforward_Blend = 0.0f;
    BallContral->Feedforward_Output = Output;
    BallContral_Apply_Combined_Output(BallContral);
}

void BallContral_Set_Feedforward_Target(BallContral_t *BallContral,
                                       uint8_t Active,
                                       PID_val Baseline,
                                       PID_val Offset,
                                       PID_val Blend)
{
    BallContral->Feedforward_Target_Active = Active ? 1U : 0U;
    BallContral->Feedforward_Baseline = Baseline;
    BallContral->Feedforward_Output = Active ? Offset : 0.0f;
    BallContral->Feedforward_Blend = Active ? Blend : 0.0f;
    BallContral_Apply_Combined_Output(BallContral);
}

void BallContral_Run(BallContral_t *BallContral, PID_val Target){
    PID_val Emm;

    if(!BallContral->is_Enable) return;

    Emm = BallContral_Calculate(BallContral, Target);
    BallContral_Set_Feedback_Output(BallContral, Emm);
}

uint8_t BallContral_Get_is_Enable(BallContral_t *BallContral){
    return BallContral->is_Enable;
}

void BallContral_Start(BallContral_t *BallContral){
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Feedback_Output = 0.0f;
    BallContral->Feedforward_Output = 0.0f;
    BallContral->Feedforward_Baseline = 0.0f;
    BallContral->Feedforward_Blend = 0.0f;
    BallContral->Feedforward_Target_Active = 0U;
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
    BallContral->Hold_Integral_Frozen = 0;
    BallContral->Hold_Is_Locked = 0;
    BallContral->Hold_Lock_Frames = 0;
    BallContral->Hold_Release_Frames = 0;
    BallContral->is_Enable = 1;
}

void BallContral_Stop(BallContral_t *BallContral){
    if(BallContral->is_Enable && BallContral->Pipe_Target_Pulse != 0){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, -BallContral->Pipe_Target_Pulse);
        BallContral->Pipe_Target_Pulse = 0;
    }
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Feedback_Output = 0.0f;
    BallContral->Feedforward_Output = 0.0f;
    BallContral->Feedforward_Baseline = 0.0f;
    BallContral->Feedforward_Blend = 0.0f;
    BallContral->Feedforward_Target_Active = 0U;
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
    BallContral->Hold_Integral_Frozen = 0;
    BallContral->Hold_Is_Locked = 0;
    BallContral->Hold_Lock_Frames = 0;
    BallContral->Hold_Release_Frames = 0;
    BallContral->is_Enable = 0;
}
