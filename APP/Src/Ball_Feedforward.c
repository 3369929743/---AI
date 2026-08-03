#include "Ball_Feedforward.h"

/*
 * 参考 G431_26H 的 AccY/AccZ 前馈结构：
 * 零偏后加速度 -> 一阶低通 -> 目标管道角度 -> 三态返回。
 * JY61P 驱动已在上电时完成 50 个静止样本的 Ay 零偏校准，
 * 因此本模块不再跟踪慢速基线，避免车辆运动被学为新零点。
 */
#define BALL_FF_POLARITY                 1.0f
#define BALL_FF_GAIN_PULSE_PER_COUNT     0.22f
#define BALL_FF_FILTER_ALPHA             0.35f
#define BALL_FF_MAX_OFFSET_PULSE        60.0f
#define BALL_FF_ENTER_ACCEL_COUNT       25.0f
#define BALL_FF_EXIT_ACCEL_COUNT        10.0f
#define BALL_FF_ENTER_CONFIRM_MS         10U
#define BALL_FF_EXIT_CONFIRM_MS         100U
#define BALL_FF_RETURN_TIMEOUT_MS       350U
#define BALL_FF_RETURN_TOLERANCE_PULSE    3.0f
/* 128 像素约 5 cm，26 像素约 1 cm；越接近边界，前馈越让权给视觉 PID。 */
#define BALL_FF_POSITION_CUTOFF_PIXEL    26.0f
#define BALL_FF_IMU_TIMEOUT_MS           150U

static PID_val BallFeedforward_Abs(PID_val Value)
{
    return (Value < 0.0f) ? -Value : Value;
}

static PID_val BallFeedforward_Clamp(PID_val Value,
                                     PID_val Minimum,
                                     PID_val Maximum)
{
    if(Value > Maximum) return Maximum;
    if(Value < Minimum) return Minimum;
    return Value;
}

void BallFeedforward_Reset_Control(BallFeedforward_t *Feedforward,
                                   int32_t Current_Pipe_Pulse)
{
    if(Feedforward == 0) return;

    Feedforward->State = BALL_FEEDFORWARD_IDLE;
    Feedforward->Baseline_Pulse = (PID_val)Current_Pipe_Pulse;
    Feedforward->Baseline_Actual_Pulse = 0.0f;
    Feedforward->Target_Offset_Pulse = 0.0f;
    Feedforward->Position_Blend = 0.0f;
    Feedforward->Confirm_Tick = 0U;
    Feedforward->Return_Tick = 0U;
    Feedforward->Enter_Confirming = 0U;
    Feedforward->Exit_Confirming = 0U;
}

void BallFeedforward_Init(BallFeedforward_t *Feedforward)
{
    if(Feedforward == 0) return;

    Feedforward->Raw_Acceleration = 0.0f;
    Feedforward->Filtered_Acceleration = 0.0f;
    Feedforward->Last_Sample_Tick = 0U;
    Feedforward->Has_Sample = 0U;
    BallFeedforward_Reset_Control(Feedforward, 0);
}

void BallFeedforward_Update(BallFeedforward_t *Feedforward,
                            int32_t Acceleration,
                            PID_val Position_Error,
                            int32_t Current_Pipe_Pulse,
                            PID_val Actual_Pipe_Pulse,
                            uint8_t Position_Feedback_Valid,
                            uint8_t Enable,
                            uint32_t Now)
{
    PID_val Acceleration_Abs;
    PID_val Pipe_Error;

    if(Feedforward == 0) return;

    Feedforward->Raw_Acceleration = (PID_val)Acceleration;
    if(!Feedforward->Has_Sample
       || (uint32_t)(Now - Feedforward->Last_Sample_Tick)
          > BALL_FF_IMU_TIMEOUT_MS){
        Feedforward->Filtered_Acceleration =
            Feedforward->Raw_Acceleration;
        Feedforward->Has_Sample = 1U;
        Feedforward->Last_Sample_Tick = Now;
        BallFeedforward_Reset_Control(Feedforward, Current_Pipe_Pulse);
        return;
    }

    Feedforward->Last_Sample_Tick = Now;
    Feedforward->Filtered_Acceleration += BALL_FF_FILTER_ALPHA
        * (Feedforward->Raw_Acceleration
           - Feedforward->Filtered_Acceleration);

    if(!Enable || !Position_Feedback_Valid){
        BallFeedforward_Reset_Control(Feedforward, Current_Pipe_Pulse);
        return;
    }

    Feedforward->Position_Blend = BallFeedforward_Clamp(
        1.0f - BallFeedforward_Abs(Position_Error)
               / BALL_FF_POSITION_CUTOFF_PIXEL,
        0.0f,
        1.0f);
    Acceleration_Abs =
        BallFeedforward_Abs(Feedforward->Filtered_Acceleration);

    if(Feedforward->State == BALL_FEEDFORWARD_IDLE){
        Feedforward->Target_Offset_Pulse = 0.0f;
        Feedforward->Position_Blend = 0.0f;

        /*
         * 钢球已经超出视觉保护范围时，禁止启动前馈。
         * 否则会出现“状态显示 ACTIVE、实际权重为 0”，同时错误地冻结保持积分。
         */
        if(BallFeedforward_Abs(Position_Error)
           >= BALL_FF_POSITION_CUTOFF_PIXEL){
            Feedforward->Enter_Confirming = 0U;
        }
        else if(Acceleration_Abs >= BALL_FF_ENTER_ACCEL_COUNT){
            if(!Feedforward->Enter_Confirming){
                /* 在加速度边沿刚出现时记录启停前的管道基准。 */
                Feedforward->Baseline_Pulse =
                    (PID_val)Current_Pipe_Pulse;
                Feedforward->Baseline_Actual_Pulse = Actual_Pipe_Pulse;
                Feedforward->Confirm_Tick = Now;
                Feedforward->Enter_Confirming = 1U;
            }
            else if((uint32_t)(Now - Feedforward->Confirm_Tick)
                    >= BALL_FF_ENTER_CONFIRM_MS){
                Feedforward->State = BALL_FEEDFORWARD_ACTIVE;
                Feedforward->Enter_Confirming = 0U;
            }
        }
        else{
            Feedforward->Enter_Confirming = 0U;
        }
    }

    if(Feedforward->State == BALL_FEEDFORWARD_ACTIVE){
        Feedforward->Position_Blend = BallFeedforward_Clamp(
            1.0f - BallFeedforward_Abs(Position_Error)
                   / BALL_FF_POSITION_CUTOFF_PIXEL,
            0.0f,
            1.0f);
        if(Feedforward->Position_Blend <= 0.0f){
            /* 偏差到达保护边界后立即让视觉 PID 完整接管。 */
            BallFeedforward_Reset_Control(Feedforward,
                                          Current_Pipe_Pulse);
            return;
        }
        Feedforward->Target_Offset_Pulse = BallFeedforward_Clamp(
            BALL_FF_POLARITY
                * BALL_FF_GAIN_PULSE_PER_COUNT
                * Feedforward->Filtered_Acceleration,
            -BALL_FF_MAX_OFFSET_PULSE,
             BALL_FF_MAX_OFFSET_PULSE);

        if(Acceleration_Abs <= BALL_FF_EXIT_ACCEL_COUNT){
            if(!Feedforward->Exit_Confirming){
                Feedforward->Confirm_Tick = Now;
                Feedforward->Exit_Confirming = 1U;
            }
            else if((uint32_t)(Now - Feedforward->Confirm_Tick)
                    >= BALL_FF_EXIT_CONFIRM_MS){
                Feedforward->State = BALL_FEEDFORWARD_RETURNING;
                Feedforward->Target_Offset_Pulse = 0.0f;
                Feedforward->Return_Tick = Now;
                Feedforward->Exit_Confirming = 0U;
            }
        }
        else{
            Feedforward->Exit_Confirming = 0U;
        }
    }
    else if(Feedforward->State == BALL_FEEDFORWARD_RETURNING){
        /* 明确回到本次启停前的管道基准，不对加速度做无限积分。 */
        Feedforward->Target_Offset_Pulse = 0.0f;
        Pipe_Error = Feedforward->Baseline_Actual_Pulse
                   - Actual_Pipe_Pulse;
        if(Feedforward->Position_Blend <= 0.0f
           || BallFeedforward_Abs(Pipe_Error)
              <= BALL_FF_RETURN_TOLERANCE_PULSE
           || (uint32_t)(Now - Feedforward->Return_Tick)
              >= BALL_FF_RETURN_TIMEOUT_MS){
            BallFeedforward_Reset_Control(Feedforward,
                                          Current_Pipe_Pulse);
        }
    }
}

uint8_t BallFeedforward_Check_Timeout(BallFeedforward_t *Feedforward,
                                      int32_t Current_Pipe_Pulse,
                                      uint32_t Now)
{
    uint8_t Was_Controlling;

    if(Feedforward == 0 || !Feedforward->Has_Sample) return 0U;
    if((uint32_t)(Now - Feedforward->Last_Sample_Tick)
       <= BALL_FF_IMU_TIMEOUT_MS){
        return 0U;
    }

    Was_Controlling =
        (uint8_t)(Feedforward->State != BALL_FEEDFORWARD_IDLE);
    Feedforward->Has_Sample = 0U;
    Feedforward->Raw_Acceleration = 0.0f;
    Feedforward->Filtered_Acceleration = 0.0f;
    BallFeedforward_Reset_Control(Feedforward, Current_Pipe_Pulse);
    return Was_Controlling;
}
