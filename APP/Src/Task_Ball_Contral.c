#include "Task_Ball_Contral.h"
#include "K230.h"
#include "Serial.h"
#include "Ball_Contral.h"
#include "Ball_Feedforward.h"
#include "SoftTimer.h"

/*
 * 5 cm 移动目标对应的视觉像素数，在这里修改移动距离。
 * 正向目标 = 本次起点 + BALL_5CM_OFFSET；
 * 负向目标 = 本次起点 - BALL_5CM_OFFSET。
 */
#define BALL_5CM_OFFSET                 128.0f

/* 到达正向 5 cm 目标的进入门限：误差不超过 6 像素时开始确认。 */
#define BALL_TARGET_TOLERANCE           6.0f
/*
 * 开始确认后的释放门限：下一帧只要仍在目标前 10 像素以内就保持确认。
 * 该滞环允许钢球经过目标时有少量帧间抖动，不要求它停在目标点。
 */
#define BALL_TARGET_CONFIRM_TOLERANCE   10.0f
/* 连续两帧确认到位，可滤除单帧跳点，同时避免确认时间过长。 */
#define BALL_TARGET_CONFIRM_FRAMES      2U

/* 到达负向 5 cm 目标的像素死区：误差不超过 3 像素即判定稳定。 */
#define BALL_MINUS_SETTLE_TOLERANCE     8.0f

/* 负向目标稳定检测：相邻视觉帧的位置变化不超过 3 像素。 */
#define BALL_MINUS_SETTLE_DELTA         3.0f
#define BALL_MINUS_SETTLE_SPEED         25.0f
#define BALL_MINUS_SETTLE_MS            650U
#define BALL_MINUS_SETTLE_FRAMES        8U
/*
 * 负向目标到位后的释放滞环：再次偏离超过 6 像素才恢复调控。
 * 该值应大于 BALL_MINUS_SETTLE_TOLERANCE，避免在死区边缘反复启停。
 */
#define BALL_MINUS_RELEASE_TOLERANCE    6.0f
#define BALL_MINUS_RELEASE_SPEED        50.0f
#define BALL_MINUS_RELEASE_FRAMES       3U
#define BALL_MINUS_TRIM_WINDOW          24.0f
#define BALL_MINUS_TRIM_SETTLE_MS       250U
#define BALL_MINUS_TRIM_GAIN            0.6f
#define BALL_MINUS_TRIM_STEP_MAX        6.0f
#define BALL_MINUS_TRIM_LIMIT           28.0f
#define BALL_ENDPOINT_MIN_PROGRESS      100.0f
/* 正向目标兜底判断使用的稳定带，不是 PID 的位置死区。 */
#define BALL_ENDPOINT_STABLE_BAND       3.0f
#define BALL_ENDPOINT_STABLE_MS         200U
/* 已有 2 s 超时处理，默认关闭“未到目标但静止也算到位”的提前结束逻辑。 */
#define BALL_ENDPOINT_STABLE_FALLBACK_ENABLE 0U
#define K230_FRAME_TIMEOUT_MS           250U

/* 正向 5 cm 运动超时时间：超过 2 s 后自动切换到负向 5 cm。 */
#define BALL_PLUS_5CM_TIMEOUT_MS        2000U
/*
 * 正向 5 cm 超时后，负向视觉目标距离的减小量，单位：像素。
 * 当前设置为 20：超时折返目标 = 本次起点 - (128 - 20) = 本次起点 - 108。
 */
#define BALL_TIMEOUT_MINUS_PIXEL_REDUCTION 20.0f

/* 电机0x36实时位置查询周期与允许年龄；前馈只在真实位置反馈新鲜时启用。 */
#define BALL_MOTOR_POSITION_QUERY_MS      50U
#define BALL_MOTOR_POSITION_MAX_AGE_MS   200U

typedef enum{
    TASK_BALL_TRAJECTORY_IDLE = 0,
    TASK_BALL_TRAJECTORY_WAIT_ORIGIN,
    TASK_BALL_TRAJECTORY_TO_PLUS_5CM,
    TASK_BALL_TRAJECTORY_TO_MINUS_5CM,
    TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM,
}Task_Ball_Trajectory_State_e;

static Serial_t Serial_K230;
static Serial_t Serial_Emm_Ball;
static SoftTimer_t SoftTimer_K230;
static SoftTimer_t SoftTimer_Ball_Plus_5cm;

static BallContral_t BallContral;

/* 5 cm position trajectory: keep its original damping parameters. */
static PID_Confg_t PID_Ball_5cm_Confg = {
    .Kp = 0.55,
    .Ki = 0.3,
    .Kd = 0.25,
    .IntMax = 40,
    .IntMin = -40,
    .OutMax = 220,
    .Alpha = 1.0
};

/* 原点保持 PID，与独立的 5 cm PID 分开配置。 */
static PID_Confg_t PID_Ball_Hold_Confg = {
    .Kp = 1.5,
    .Ki = 0.45,
    .Kd = 0.35,
    .IntMax = 20,
    .IntMin = -20,
    .OutMax = 220,
    .Alpha = 1.0
};

static Task_Ball_Contral_State_e Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
static Task_Ball_Trajectory_State_e Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_IDLE;
static PID_val Task_Ball_Origin_x = 0.0f;
static uint32_t Task_Ball_Last_Vision_Frame_Tick = 0;
static PID_val Task_Ball_Endpoint_Anchor_x = 0.0f;
static uint32_t Task_Ball_Endpoint_Stable_Tick = 0;
static uint8_t Task_Ball_Endpoint_Tracking = 0;
static uint8_t Task_Ball_Target_Confirm_Frames = 0;
static uint32_t Task_Ball_Minus_Settle_Tick = 0;
static uint8_t Task_Ball_Minus_Settle_Frames = 0;
static uint8_t Task_Ball_Minus_Release_Frames = 0;
static PID_val Task_Ball_Plus_Endpoint_x = 0.0f;
static PID_val Task_Ball_Minus_Final_Target_x = 0.0f;
static PID_val Task_Ball_Minus_Target_Trim = 0.0f;
static PID_val Task_Ball_Minus_Pixel_Reduction = 0.0f;
static uint8_t Task_Ball_Use_Position_Hold = 0;
static BallFeedforward_t Task_Ball_Feedforward;
static PID_val Task_Ball_Motor_Actual_Pulse = 0.0f;
static PID_val Task_Ball_Motor_Position_Origin = 0.0f;
static uint32_t Task_Ball_Motor_Position_Age = 0xFFFFFFFFUL;
static uint8_t Task_Ball_Motor_Position_Valid = 0U;
static uint8_t Task_Ball_Motor_Position_Has_Origin = 0U;

static uint8_t Task_Ball_IMU_Feedforward_Can_Apply(void){
    return Task_Ball_Use_Position_Hold
        && Task_Ball_Contral_State == TASK_BALL_CONTRAL_RUNNING
        && BallContral_Get_is_Enable(&BallContral)
        && K230_IsZeroReady()
        && Task_Ball_Motor_Position_Valid
        && Task_Ball_Motor_Position_Age
           <= BALL_MOTOR_POSITION_MAX_AGE_MS;
}

static void Task_Ball_IMU_Feedforward_Apply(void){
    uint8_t Active =
        (uint8_t)(Task_Ball_Feedforward.State != BALL_FEEDFORWARD_IDLE
                  && Task_Ball_Feedforward.Position_Blend > 0.0f
                  && Task_Ball_IMU_Feedforward_Can_Apply());

    BallContral.Hold_Integral_Frozen = Active;
    if(Active){
        BallContral.Hold_Is_Locked = 0U;
        BallContral.Hold_Lock_Frames = 0U;
        BallContral.Hold_Release_Frames = 0U;
    }
    BallContral_Set_Feedforward_Target(
        &BallContral,
        Active,
        Task_Ball_Feedforward.Baseline_Pulse,
        Task_Ball_Feedforward.Target_Offset_Pulse,
        Task_Ball_Feedforward.Position_Blend);
}

static void Task_Ball_IMU_Feedforward_Reset(void){
    BallFeedforward_Reset_Control(&Task_Ball_Feedforward,
                                  BallContral.Pipe_Target_Pulse);
    BallContral.Hold_Integral_Frozen = 0U;
    BallContral_Set_Feedforward_Target(&BallContral,
                                      0U, 0.0f, 0.0f, 0.0f);
}

static void Task_Ball_Motor_Position_Update(void){
    Task_Ball_Motor_Position_Valid = Emm_Get_RealTime_Position_Pulse(
        &BallContral.Emm_StepMotor,
        &Task_Ball_Motor_Actual_Pulse,
        &Task_Ball_Motor_Position_Age);
    if(Task_Ball_Motor_Position_Valid
       && !Task_Ball_Motor_Position_Has_Origin){
        /* 对齐驱动器绝对位置与本工程从 0 开始累计的管道目标。 */
        Task_Ball_Motor_Position_Origin =
            Task_Ball_Motor_Actual_Pulse
            - (PID_val)BallContral.Pipe_Target_Pulse;
        Task_Ball_Motor_Position_Has_Origin = 1U;
    }
}

static void Task_Ball_IMU_Feedforward_Timeout_Update(void){
    uint32_t Now = HAL_GetTick();

    Emm_Position_Feedback_Loop(&BallContral.Emm_StepMotor,
                               Now,
                               BALL_MOTOR_POSITION_QUERY_MS);
    Task_Ball_Motor_Position_Update();
    if(BallFeedforward_Check_Timeout(&Task_Ball_Feedforward,
                                    BallContral.Pipe_Target_Pulse,
                                    Now)){
        Task_Ball_IMU_Feedforward_Apply();
    }
    if(Task_Ball_Motor_Position_Valid
       && Task_Ball_Motor_Position_Age
          > BALL_MOTOR_POSITION_MAX_AGE_MS
       && Task_Ball_Feedforward.State != BALL_FEEDFORWARD_IDLE){
        Task_Ball_IMU_Feedforward_Reset();
    }
}

static void Task_Ball_Select_Control(uint8_t Use_Position_Hold){
    if(Task_Ball_Use_Position_Hold == Use_Position_Hold) return;

    Task_Ball_IMU_Feedforward_Reset();
    Task_Ball_Use_Position_Hold = Use_Position_Hold;
    if(Use_Position_Hold){
        PID_Init(&BallContral.PID_StepMotor, &PID_Ball_Hold_Confg);
        BallContral_Hold_Mode_Init(&BallContral);
    }
    else{
        PID_Init(&BallContral.PID_StepMotor, &PID_Ball_5cm_Confg);
        BallContral_Position_Mode_Init(&BallContral);
    }
}

static PID_val Task_Ball_Abs(PID_val Value){
    return (Value < 0.0f) ? -Value : Value;
}

static PID_val Task_Ball_Clamp(PID_val Value, PID_val Min, PID_val Max){
    if(Value > Max) return Max;
    if(Value < Min) return Min;
    return Value;
}

static void Task_Ball_Endpoint_Tracking_Reset(void){
    Task_Ball_Endpoint_Anchor_x = 0.0f;
    Task_Ball_Endpoint_Stable_Tick = 0;
    Task_Ball_Endpoint_Tracking = 0;
    Task_Ball_Target_Confirm_Frames = 0;
}

static void Task_Ball_Minus_Tracking_Reset(void){
    Task_Ball_Minus_Settle_Tick = 0;
    Task_Ball_Minus_Settle_Frames = 0;
    Task_Ball_Minus_Release_Frames = 0;
}

static PID_val Task_Ball_Minus_Base_Target(void){
    return Task_Ball_Origin_x
         - (BALL_5CM_OFFSET - Task_Ball_Minus_Pixel_Reduction);
}

static void Task_Ball_Minus_Final_Reset(void){
    Task_Ball_Plus_Endpoint_x = 0.0f;
    Task_Ball_Minus_Final_Target_x = 0.0f;
    Task_Ball_Minus_Target_Trim = 0.0f;
    Task_Ball_Minus_Pixel_Reduction = 0.0f;
    Task_Ball_Minus_Tracking_Reset();
}

static void Task_Ball_Minus_Apply_Target(void){
    BallContral_Set_Target(&BallContral,
                           Task_Ball_Minus_Base_Target() + Task_Ball_Minus_Target_Trim);
}

static void Task_Ball_Minus_Begin_Return(PID_val Plus_Endpoint,
                                         PID_val Pixel_Reduction){
    Task_Ball_Minus_Pixel_Reduction = Task_Ball_Clamp(
        Pixel_Reduction, 0.0f, BALL_5CM_OFFSET);
    Task_Ball_Plus_Endpoint_x = Plus_Endpoint;
    Task_Ball_Minus_Final_Target_x = (2.0f * Task_Ball_Origin_x) - Task_Ball_Plus_Endpoint_x;
    Task_Ball_Minus_Target_Trim = 0.0f;
    Task_Ball_Minus_Tracking_Reset();
    Task_Ball_Minus_Apply_Target();
}

static void Task_Ball_Minus_Trim_Target(PID_val Position_Error){
    PID_val Adjustment = Position_Error * BALL_MINUS_TRIM_GAIN;

    Adjustment = Task_Ball_Clamp(Adjustment,
                                 -BALL_MINUS_TRIM_STEP_MAX,
                                  BALL_MINUS_TRIM_STEP_MAX);
    Task_Ball_Minus_Target_Trim += Adjustment;
    Task_Ball_Minus_Target_Trim = Task_Ball_Clamp(Task_Ball_Minus_Target_Trim,
                                                  -BALL_MINUS_TRIM_LIMIT,
                                                   BALL_MINUS_TRIM_LIMIT);
    BallContral_Clear_Integral(&BallContral);
    Task_Ball_Minus_Apply_Target();
}

static void Task_Ball_Trajectory_Cancel(void){
    SoftTimer_Stop(&SoftTimer_Ball_Plus_5cm);
    BallContral_Set_Output_Pulse_Reduction(&BallContral, 0.0f);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_IDLE;
    Task_Ball_Endpoint_Tracking_Reset();
    Task_Ball_Minus_Final_Reset();
}

static uint8_t Task_Ball_Minus5cm_Is_Stable(PID_val Position){
    PID_val Position_Error = Task_Ball_Minus_Final_Target_x - Position;
    PID_val Error = Task_Ball_Abs(Position_Error);
    PID_val Frame_Delta = Task_Ball_Abs(Position - BallContral.Ball_Position_Pre);
    PID_val Speed = Task_Ball_Abs(BallContral.Ball_Velocity);
    uint32_t Now = HAL_GetTick();
    uint32_t Required_Settle_ms;

    if(!BallContral.Has_Ball_History
       || Error > BALL_MINUS_TRIM_WINDOW
       || Frame_Delta > BALL_MINUS_SETTLE_DELTA
       || Speed > BALL_MINUS_SETTLE_SPEED){
        Task_Ball_Minus_Settle_Tick = 0;
        Task_Ball_Minus_Settle_Frames = 0;
        return 0;
    }

    if(Task_Ball_Minus_Settle_Frames == 0U){
        Task_Ball_Minus_Settle_Tick = Now;
    }
    if(Task_Ball_Minus_Settle_Frames < 0xFFU){
        Task_Ball_Minus_Settle_Frames++;
    }

    Required_Settle_ms = (Error <= BALL_MINUS_SETTLE_TOLERANCE) ?
                         BALL_MINUS_SETTLE_MS :
                         BALL_MINUS_TRIM_SETTLE_MS;
    if(Task_Ball_Minus_Settle_Frames < BALL_MINUS_SETTLE_FRAMES
       || (uint32_t)(Now - Task_Ball_Minus_Settle_Tick) < Required_Settle_ms){
        return 0;
    }

    if(Error <= BALL_MINUS_SETTLE_TOLERANCE){
        return 1;
    }

    Task_Ball_Minus_Trim_Target(Position_Error);
    Task_Ball_Minus_Tracking_Reset();
    return 0;
}

static uint8_t Task_Ball_Minus5cm_Hold_Is_Lost(PID_val Position){
    PID_val Target = Task_Ball_Minus_Final_Target_x;
    PID_val Error = Task_Ball_Abs(Position - Target);
    PID_val Speed = Task_Ball_Abs(BallContral.Ball_Velocity);

    if(Error > BALL_MINUS_RELEASE_TOLERANCE
       || Speed > BALL_MINUS_RELEASE_SPEED){
        if(Task_Ball_Minus_Release_Frames < 0xFFU){
            Task_Ball_Minus_Release_Frames++;
        }
    }
    else{
        Task_Ball_Minus_Release_Frames = 0;
    }

    return Task_Ball_Minus_Release_Frames >= BALL_MINUS_RELEASE_FRAMES;
}

static uint8_t Task_Ball_Target_Is_Reached(PID_val Position, PID_val Target,
                                           int8_t Direction){
    PID_val Progress;
    PID_val Anchor_Error;
    PID_val Active_Tolerance;
    uint8_t In_Target_Band;

    Active_Tolerance = (Task_Ball_Target_Confirm_Frames == 0U) ?
                       BALL_TARGET_TOLERANCE :
                       BALL_TARGET_CONFIRM_TOLERANCE;

    if(Direction > 0){
        In_Target_Band = Position >= Target - Active_Tolerance;
        Progress = Position - Task_Ball_Origin_x;
    }
    else{
        In_Target_Band = Position <= Target + Active_Tolerance;
        Progress = Task_Ball_Origin_x - Position;
    }

    if(In_Target_Band){
        /* 到位必须由连续有效视觉帧确认，单帧跳点不会结束轨迹。 */
        Task_Ball_Endpoint_Anchor_x = 0.0f;
        Task_Ball_Endpoint_Stable_Tick = 0;
        Task_Ball_Endpoint_Tracking = 0;
        if(Task_Ball_Target_Confirm_Frames < BALL_TARGET_CONFIRM_FRAMES){
            Task_Ball_Target_Confirm_Frames++;
        }
        if(Task_Ball_Target_Confirm_Frames >= BALL_TARGET_CONFIRM_FRAMES){
            Task_Ball_Endpoint_Tracking_Reset();
            return 1;
        }
        return 0;
    }
    Task_Ball_Target_Confirm_Frames = 0;

    /* 禁用后必须真正进入目标死区；卡住时统一交给 2 s 超时处理。 */
    if(!BALL_ENDPOINT_STABLE_FALLBACK_ENABLE){
        Task_Ball_Endpoint_Tracking_Reset();
        return 0;
    }

    if(Progress < BALL_ENDPOINT_MIN_PROGRESS){
        Task_Ball_Endpoint_Tracking_Reset();
        return 0;
    }

    if(!Task_Ball_Endpoint_Tracking){
        Task_Ball_Endpoint_Anchor_x = Position;
        Task_Ball_Endpoint_Stable_Tick = HAL_GetTick();
        Task_Ball_Endpoint_Tracking = 1;
        return 0;
    }

    Anchor_Error = Position - Task_Ball_Endpoint_Anchor_x;
    if(Anchor_Error < 0.0f) Anchor_Error = -Anchor_Error;
    if(Anchor_Error > BALL_ENDPOINT_STABLE_BAND){
        Task_Ball_Endpoint_Anchor_x = Position;
        Task_Ball_Endpoint_Stable_Tick = HAL_GetTick();
        return 0;
    }

    if((uint32_t)(HAL_GetTick() - Task_Ball_Endpoint_Stable_Tick) >=
       BALL_ENDPOINT_STABLE_MS){
        Task_Ball_Endpoint_Tracking_Reset();
        return 1;
    }
    return 0;
}

static void Task_Ball_Trajectory_Prepare(PID_val Position){
    if(Task_Ball_Trajectory_State != TASK_BALL_TRAJECTORY_WAIT_ORIGIN) return;

    /* Capture the origin only from a newly decoded vision frame. */
    Task_Ball_Origin_x = Position;
    Task_Ball_Endpoint_Tracking_Reset();
    Task_Ball_Minus_Final_Reset();
    BallContral_Set_Output_Pulse_Reduction(&BallContral, 0.0f);
    BallContral_Start(&BallContral);
    BallContral_Set_Target(&BallContral, Task_Ball_Origin_x + BALL_5CM_OFFSET);
    SoftTimer_Reset(&SoftTimer_Ball_Plus_5cm);
    SoftTimer_Start(&SoftTimer_Ball_Plus_5cm);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_PLUS_5CM;
}

static void Task_Ball_Trajectory_Update(PID_val Position){
    switch(Task_Ball_Trajectory_State){
        case TASK_BALL_TRAJECTORY_TO_PLUS_5CM:
            if(Task_Ball_Target_Is_Reached(Position,
                                           Task_Ball_Origin_x + BALL_5CM_OFFSET,
                                           1)){
                SoftTimer_Stop(&SoftTimer_Ball_Plus_5cm);
                /* 正常到达后折返不减脉冲。 */
                BallContral_Set_Output_Pulse_Reduction(&BallContral, 0.0f);
                /* Keep velocity history for a smooth reversal, but discard old integral bias. */
                BallContral_Clear_Integral(&BallContral);
                Task_Ball_Minus_Begin_Return(Position, 0.0f);
                Task_Ball_Endpoint_Tracking_Reset();
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
            }
            break;
        case TASK_BALL_TRAJECTORY_TO_MINUS_5CM:
            if(Task_Ball_Minus5cm_Is_Stable(Position)){
                Task_Ball_Minus_Release_Frames = 0;
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM;
            }
            break;
        case TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM:
            /* Keep the learned integral bias; it is needed to remove final static error. */
            if(Task_Ball_Minus5cm_Hold_Is_Lost(Position)){
                Task_Ball_Minus_Tracking_Reset();
                BallContral_Clear_Integral(&BallContral);
                Task_Ball_Minus_Apply_Target();
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
            }
            break;
        case TASK_BALL_TRAJECTORY_IDLE:
        case TASK_BALL_TRAJECTORY_WAIT_ORIGIN:
        default:
            break;
    }
}

static void Task_Ball_Plus5cm_Timeout_Update(void){
    if(Task_Ball_Trajectory_State != TASK_BALL_TRAJECTORY_TO_PLUS_5CM) return;
    if(!SoftTimer_Trigger(&SoftTimer_Ball_Plus_5cm)) return;

    /*
     * 正向运动超过 2 s：放弃继续等待正向到位，直接切换到
     * 以本次起点为基准、减去像素补偿量后的负向目标。
     */
    BallContral_Clear_Integral(&BallContral);
    /*
     * 超时只缩短负向视觉目标，不再削减电机 PID 输出脉冲。
     * 传入等效正端点，使负端最终目标与缩短后的 PID 目标保持一致。
     */
    Task_Ball_Minus_Begin_Return(
        Task_Ball_Origin_x + BALL_5CM_OFFSET
                           - BALL_TIMEOUT_MINUS_PIXEL_REDUCTION,
        BALL_TIMEOUT_MINUS_PIXEL_REDUCTION);
    Task_Ball_Endpoint_Tracking_Reset();
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
}

static void Task_Ball_Process_New_Frame(void){
    PID_val Position = (PID_val)K230_GetError_x();

    Task_Ball_Trajectory_Prepare(Position);
    /* Switch the target before calculating this frame's PID output. */
    Task_Ball_Trajectory_Update(Position);
    if(Task_Ball_Use_Position_Hold){
        BallContral_Run_Position_Hold(&BallContral, Position);
    }
    else{
        BallContral_Run(&BallContral, Position);
    }
}

void Task_Ball_Contral_Init(void){
    Serial_Init(&Serial_K230, Serial_3);
    Serial_Init(&Serial_Emm_Ball, Serial_1);
    BallContral_Init(&BallContral, &Serial_K230, &Serial_Emm_Ball,
                     &PID_Ball_5cm_Confg);
    K230_Init(&Serial_K230, 0, 0);

    SoftTimer_Init(&SoftTimer_K230, SOFTTIMER_MODE_PERIODIC, K230_FRAME_TIMEOUT_MS);
    SoftTimer_Init(&SoftTimer_Ball_Plus_5cm,
                   SOFTTIMER_MODE_SINGLE,
                   BALL_PLUS_5CM_TIMEOUT_MS);

    SoftTimer_Start(&SoftTimer_K230);

    BallContral_Set_Target(&BallContral, 0);
    Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
    Task_Ball_Last_Vision_Frame_Tick = HAL_GetTick();
    BallFeedforward_Init(&Task_Ball_Feedforward);
    Task_Ball_Motor_Actual_Pulse = 0.0f;
    Task_Ball_Motor_Position_Origin = 0.0f;
    Task_Ball_Motor_Position_Age = 0xFFFFFFFFUL;
    Task_Ball_Motor_Position_Valid = 0U;
    Task_Ball_Motor_Position_Has_Origin = 0U;
    Task_Ball_IMU_Feedforward_Reset();
    Task_Ball_Trajectory_Cancel();
}

void Task_Ball_Contral_Toggle(void){
    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        Task_Ball_Trajectory_Cancel();
        /* 丢弃按键前缓存的旧帧，确保下一帧才是真正的上电/启动原点。 */
        (void)K230_GetFlag();
        K230_ResetZero();
        Task_Ball_Select_Control(1);
        Task_Ball_IMU_Feedforward_Reset();
        SoftTimer_Reset(&SoftTimer_K230);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
        BallContral_Start(&BallContral);
    }
    else{
        Task_Ball_Trajectory_Cancel();
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
        BallContral_Stop(&BallContral);
        Task_Ball_IMU_Feedforward_Reset();
    }
}

void Task_Ball_Contral_Loop(void){
    uint8_t Has_New_Frame = 0;

    Task_Ball_IMU_Feedforward_Timeout_Update();

    switch(Task_Ball_Contral_State){
        case TASK_BALL_CONTRAL_IDLE:
            break;
        case TASK_BALL_CONTRAL_RUNNING:
            if(K230_Error_Update()){
                Task_Ball_Last_Vision_Frame_Tick = HAL_GetTick();
                SoftTimer_Reset(&SoftTimer_K230);
                Has_New_Frame = 1;
            }
            if(SoftTimer_Trigger(&SoftTimer_K230)){
                Task_Ball_Endpoint_Tracking_Reset();
                Task_Ball_Minus_Tracking_Reset();
                if(Task_Ball_Trajectory_State == TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM){
                    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
                }
                BallContral_Stop(&BallContral);
                Task_Ball_IMU_Feedforward_Reset();
                Task_Ball_Contral_State = TASK_BALL_CONTRAL_LOST;
            }
            else{
                Task_Ball_Plus5cm_Timeout_Update();
                if(Has_New_Frame){
                    Task_Ball_Process_New_Frame();
                }
            }
            break;
        case TASK_BALL_CONTRAL_LOST:
            BallContral_Stop(&BallContral);
            if(K230_Error_Update()){
                Task_Ball_Last_Vision_Frame_Tick = HAL_GetTick();
                SoftTimer_Reset(&SoftTimer_K230);
                BallContral_Start(&BallContral);
                Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
                Task_Ball_Process_New_Frame();
            }
            break;
    }
}

void Task_Ball_Contral_Update_Acceleration(int32_t AccelAy){
    uint32_t Now = HAL_GetTick();
    PID_val Position_Error = (PID_val)K230_GetError_x()
                           - BallContral.Hold_Target;

    Task_Ball_Motor_Position_Update();
    BallFeedforward_Update(
        &Task_Ball_Feedforward,
        AccelAy,
        Position_Error,
        BallContral.Pipe_Target_Pulse,
        Task_Ball_Motor_Actual_Pulse,
        (uint8_t)(Task_Ball_Motor_Position_Valid
                  && Task_Ball_Motor_Position_Age
                     <= BALL_MOTOR_POSITION_MAX_AGE_MS),
        Task_Ball_IMU_Feedforward_Can_Apply(),
        Now);
    Task_Ball_IMU_Feedforward_Apply();
}

float Task_Ball_Contral_Get_IMU_Feedforward(void){
    return (float)BallContral.Feedforward_Output;
}

int32_t Task_Ball_Contral_Get_IMU_Accel_Filtered(void){
    PID_val Accel = Task_Ball_Feedforward.Filtered_Acceleration;

    return (Accel >= 0.0f) ?
           (int32_t)(Accel + 0.5f) :
           (int32_t)(Accel - 0.5f);
}

static int32_t Task_Ball_Debug_Round_x10(PID_val Value){
    Value *= 10.0f;
    return (Value >= 0.0f) ?
           (int32_t)(Value + 0.5f) :
           (int32_t)(Value - 0.5f);
}

void Task_Ball_Contral_Get_Debug_Data(Task_Ball_Debug_Data_t *Debug_Data){
    uint8_t Flags = 0;
    PID_val Combined_Output;

    if(Debug_Data == 0) return;

    if(Task_Ball_Feedforward.State == BALL_FEEDFORWARD_ACTIVE) Flags |= 0x01U;
    if(Task_Ball_Feedforward.State == BALL_FEEDFORWARD_RETURNING) Flags |= 0x02U;
    if(BallContral.Hold_Is_Locked) Flags |= 0x04U;
    if(K230_IsZeroReady()) Flags |= 0x08U;
    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_RUNNING) Flags |= 0x10U;
    if(Task_Ball_Use_Position_Hold) Flags |= 0x20U;
    if(Task_Ball_Feedforward.Has_Sample) Flags |= 0x40U;
    if(Task_Ball_Motor_Position_Valid
       && Task_Ball_Motor_Position_Age
          <= BALL_MOTOR_POSITION_MAX_AGE_MS) Flags |= 0x80U;

    Debug_Data->Accel_Ay_Filtered =
        Task_Ball_Contral_Get_IMU_Accel_Filtered();
    Debug_Data->Accel_Ay_Highpass =
        (Task_Ball_Feedforward.Raw_Acceleration >= 0.0f) ?
        (int32_t)(Task_Ball_Feedforward.Raw_Acceleration + 0.5f) :
        (int32_t)(Task_Ball_Feedforward.Raw_Acceleration - 0.5f);
    Debug_Data->Pipe_Target_Pulse = BallContral.Pipe_Target_Pulse;
    Debug_Data->Feedforward_x10 =
        Task_Ball_Debug_Round_x10(BallContral.Feedforward_Output);
    Debug_Data->Feedback_x10 =
        Task_Ball_Debug_Round_x10(BallContral.Feedback_Output);
    if(BallContral.Feedforward_Target_Active){
        Combined_Output = BallContral.Feedback_Output
            + BallContral.Feedforward_Blend
              * (BallContral.Feedforward_Baseline
                 + BallContral.Feedforward_Output
                 - BallContral.Feedback_Output);
    }
    else{
        Combined_Output = BallContral.Feedback_Output
                        + BallContral.Feedforward_Output;
    }
    Debug_Data->Combined_x10 =
        Task_Ball_Debug_Round_x10(Combined_Output);
    Debug_Data->Integral_x10 =
        Task_Ball_Debug_Round_x10(BallContral.PID_StepMotor.ErrorInt);
    if(Task_Ball_Motor_Position_Has_Origin){
        PID_val Motor_Actual_Relative = Task_Ball_Motor_Actual_Pulse
                                      - Task_Ball_Motor_Position_Origin;
        Debug_Data->Motor_Actual_Pulse =
            (Motor_Actual_Relative >= 0.0f) ?
            (int32_t)(Motor_Actual_Relative + 0.5f) :
            (int32_t)(Motor_Actual_Relative - 0.5f);
    }
    else{
        Debug_Data->Motor_Actual_Pulse = 0;
    }
    Debug_Data->Motor_Position_Age_ms =
        (Task_Ball_Motor_Position_Age <= 99999UL) ?
        Task_Ball_Motor_Position_Age : 99999UL;
    Debug_Data->Feedforward_Blend_x100 =
        (uint16_t)(Task_Ball_Feedforward.Position_Blend * 100.0f + 0.5f);
    Debug_Data->Vision_Zero_x = K230_GetZero_x();
    Debug_Data->Vision_Error_x = K230_GetError_x();
    Debug_Data->Target_x = (int16_t)BallContral.PID_StepMotor.Target;
    Debug_Data->Flags = Flags;
}

void Task_Ball_Contral_Tick(void){
    SoftTimer_Update(&SoftTimer_K230);
    SoftTimer_Update(&SoftTimer_Ball_Plus_5cm);
}

void Task_Ball_Contral_Pop_Init(void){
    Ball_Contral_Emm_Quick_Init(&BallContral);
}

void Task_Ball_Contral_Pop_Ready(void){
    Ball_Contral_Pop_Run(&BallContral, -100);
}

void Task_Ball_Contral_Pop_Restore(void){
    Ball_Contral_Pop_Run(&BallContral, 100);
}

void Task_Ball_Goto5cm(void){
    Task_Ball_Trajectory_Cancel();
    Task_Ball_Select_Control(0);
    BallContral_Set_Target(&BallContral, BALL_5CM_OFFSET);
}

void Task_Ball_GotoMinus5cm(void){
    Task_Ball_Trajectory_Cancel();
    Task_Ball_Select_Control(0);
    BallContral_Set_Target(&BallContral, -BALL_5CM_OFFSET);
}

void Task_Ball_Start_5cm_Sequence(void){
    /* Discard a frame buffered before the trigger, then wait for a fresh origin. */
    Task_Ball_Select_Control(0);
    SoftTimer_Stop(&SoftTimer_Ball_Plus_5cm);
    (void)K230_GetFlag();
    SoftTimer_Reset(&SoftTimer_K230);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_WAIT_ORIGIN;
    Task_Ball_Endpoint_Tracking_Reset();
    Task_Ball_Minus_Final_Reset();

    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        BallContral_Start(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
    }
}

/* Compatibility API retained for an explicit idle-state zero reset. */
void Task_Ball_Reset_Zero(void){
    /*
     * 保持运行中禁止重采视觉零点，任何误调用都不能把大幅启停后的
     * 偏移位置写成新的目标点。
     */
    if(Task_Ball_Contral_State != TASK_BALL_CONTRAL_IDLE) return;

    Task_Ball_Trajectory_Cancel();
    Task_Ball_Select_Control(1);
    Task_Ball_IMU_Feedforward_Reset();
    /* 只允许按键之后到达的新视觉帧成为零点。 */
    (void)K230_GetFlag();
    K230_ResetZero();
    SoftTimer_Reset(&SoftTimer_K230);
    BallContral_Clear_Integral(&BallContral);
    BallContral_Set_Target(&BallContral, 0);
}

uint8_t Task_Ball_Get_Control_State(void){
    return (uint8_t)Task_Ball_Contral_State;
}

uint8_t Task_Ball_Get_Trajectory_State(void){
    return (uint8_t)Task_Ball_Trajectory_State;
}

uint32_t Task_Ball_Get_Vision_Frame_Age(void){
    return HAL_GetTick() - Task_Ball_Last_Vision_Frame_Tick;
}

int16_t Task_Ball_Get_Target_x(void){
    return (int16_t)BallContral.PID_StepMotor.Target;
}
