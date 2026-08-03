#include "Task_Ball_Contral.h"
#include "K230.h"
#include "Serial.h"
#include "Ball_Contral.h"
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

/*
 * 小车启停事件前馈参数（不使用陀螺仪/加速度计）。
 * 当前 138 像素约等于 5 cm，因此 1 cm 误差约为 28 像素。
 *
 */
#define BALL_CAR_FF_POLARITY            (1.0f)   /**< 前馈方向极性，若启动后误差反而增大，改为 -1.0f */
#define BALL_CAR_START_FF_PULSE         25.0f    /**< 启动瞬间附加的管道目标脉冲 */
#define BALL_CAR_STOP_FF_PULSE          25.0f    /**< 停止瞬间附加的管道目标脉冲 */
#define BALL_CAR_START_FF_HOLD_MS       200U     /**< 启动前馈峰值维持时间(ms) */
#define BALL_CAR_START_FF_FADE_MS       300U     /**< 启动前馈从峰值衰减到0的时间(ms) */
#define BALL_CAR_STOP_FF_HOLD_MS        150U     /**< 停止前馈峰值维持时间(ms) */
#define BALL_CAR_STOP_FF_FADE_MS        300U     /**< 停止前馈从峰值衰减到0的时间(ms) */
#define BALL_CAR_FF_OUTPUT_LIMIT        60.0f    /**< 前馈输出上限 */
#define BALL_CAR_FF_MIN_UPDATE          0.5f     /**< 前馈最小更新间隔(s) */

#define BALL_CAR_FF_READY_DELAY_MS       0U

typedef enum{
    TASK_BALL_TRAJECTORY_IDLE = 0,
    TASK_BALL_TRAJECTORY_WAIT_ORIGIN,
    TASK_BALL_TRAJECTORY_TO_PLUS_5CM,
    TASK_BALL_TRAJECTORY_TO_MINUS_5CM,
    TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM,
}Task_Ball_Trajectory_State_e;

typedef enum{
    TASK_BALL_CAR_FF_IDLE = 0,
    TASK_BALL_CAR_FF_START,
    TASK_BALL_CAR_FF_RUN,
    TASK_BALL_CAR_FF_STOP,
}Task_Ball_Car_FF_State_e;

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

/* Position hold: filtered damping is active during escape and return. */
static PID_Confg_t PID_Ball_Hold_Confg = {
    .Kp = 1.5,
    .Ki = 0.45,
    .Kd = 0.35,
    .IntMax = 60,
    .IntMin = -60,
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
static Task_Ball_Car_FF_State_e Task_Ball_Car_FF_State = TASK_BALL_CAR_FF_IDLE;
static uint32_t Task_Ball_Car_FF_Start_Tick = 0;
static PID_val Task_Ball_Car_FF_Applied = 0.0f;
static uint8_t Task_Ball_Car_Motor_Running = 0;
static uint8_t Task_Ball_Car_Motor_State_Valid = 0;
static uint8_t Task_Ball_Car_FF_Ready_Pending = 0;
static uint32_t Task_Ball_Car_FF_Ready_Tick = 0;

static void Task_Ball_Select_Control(uint8_t Use_Position_Hold){
    if(Task_Ball_Use_Position_Hold == Use_Position_Hold) return;

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

static PID_val Task_Ball_Car_FF_Profile(PID_val Peak,
                                        uint32_t Elapsed_ms,
                                        uint32_t Hold_ms,
                                        uint32_t Fade_ms){
    if(Elapsed_ms < Hold_ms) return Peak;

    Elapsed_ms -= Hold_ms;
    if(Elapsed_ms >= Fade_ms || Fade_ms == 0U) return 0.0f;

    return Peak * (PID_val)(Fade_ms - Elapsed_ms) / (PID_val)Fade_ms;
}

static uint8_t Task_Ball_Car_Feedforward_Apply(PID_val Feedforward){
    PID_val Change;
    uint8_t Can_Apply;

    /* 5 cm 轨迹使用独立 PID，不允许小车前馈影响它。 */
    Can_Apply = Task_Ball_Use_Position_Hold
             && Task_Ball_Contral_State == TASK_BALL_CONTRAL_RUNNING
             && BallContral_Get_is_Enable(&BallContral);
    if(!Can_Apply){
        Feedforward = 0.0f;
    }

    Feedforward = Task_Ball_Clamp(Feedforward,
                                  -BALL_CAR_FF_OUTPUT_LIMIT,
                                   BALL_CAR_FF_OUTPUT_LIMIT);
    Change = Task_Ball_Abs(Feedforward - Task_Ball_Car_FF_Applied);
    if(Feedforward == 0.0f){
        if(Task_Ball_Car_FF_Applied == 0.0f) return Can_Apply;
    }
    else if(Change < BALL_CAR_FF_MIN_UPDATE){
        return Can_Apply;
    }

    Task_Ball_Car_FF_Applied = Feedforward;
    BallContral_Set_Feedforward_Output(&BallContral, Feedforward);
    return Can_Apply;
}

static uint8_t Task_Ball_Car_Feedforward_Update(void){
    uint32_t Elapsed_ms = HAL_GetTick() - Task_Ball_Car_FF_Start_Tick;
    PID_val Feedforward = 0.0f;

    switch(Task_Ball_Car_FF_State){
        case TASK_BALL_CAR_FF_START:
            Feedforward = Task_Ball_Car_FF_Profile(
                BALL_CAR_FF_POLARITY * BALL_CAR_START_FF_PULSE,
                Elapsed_ms,
                BALL_CAR_START_FF_HOLD_MS,
                BALL_CAR_START_FF_FADE_MS);
            if(Elapsed_ms >= BALL_CAR_START_FF_HOLD_MS
                           + BALL_CAR_START_FF_FADE_MS){
                Task_Ball_Car_FF_State = TASK_BALL_CAR_FF_RUN;
            }
            break;

        case TASK_BALL_CAR_FF_STOP:
            Feedforward = Task_Ball_Car_FF_Profile(
                -BALL_CAR_FF_POLARITY * BALL_CAR_STOP_FF_PULSE,
                Elapsed_ms,
                BALL_CAR_STOP_FF_HOLD_MS,
                BALL_CAR_STOP_FF_FADE_MS);
            if(Elapsed_ms >= BALL_CAR_STOP_FF_HOLD_MS
                           + BALL_CAR_STOP_FF_FADE_MS){
                Task_Ball_Car_FF_State = TASK_BALL_CAR_FF_IDLE;
            }
            break;

        case TASK_BALL_CAR_FF_IDLE:
        case TASK_BALL_CAR_FF_RUN:
        default:
            Feedforward = 0.0f;
            break;
    }

    return Task_Ball_Car_Feedforward_Apply(Feedforward);
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
    Task_Ball_Car_FF_State = TASK_BALL_CAR_FF_IDLE;
    Task_Ball_Car_FF_Applied = 0.0f;
    Task_Ball_Car_Motor_Running = 0;
    Task_Ball_Car_Motor_State_Valid = 0;
    Task_Ball_Car_FF_Ready_Pending = 0;
    Task_Ball_Car_FF_Ready_Tick = 0;
    Task_Ball_Trajectory_Cancel();
}

void Task_Ball_Contral_Toggle(void){
    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        Task_Ball_Trajectory_Cancel();
        K230_ResetZero();
        Task_Ball_Select_Control(1);
        SoftTimer_Reset(&SoftTimer_K230);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
        BallContral_Start(&BallContral);
    }
    else{
        Task_Ball_Trajectory_Cancel();
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
        BallContral_Stop(&BallContral);
    }
}

void Task_Ball_Contral_Loop(void){
    uint8_t Has_New_Frame = 0;

    Task_Ball_Car_Feedforward_Update();

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

uint8_t Task_Ball_Contral_Set_Car_Motor_State(uint8_t IsRunning){
    uint8_t Feedforward_Applied;

    IsRunning = (IsRunning != 0U) ? 1U : 0U;
    /* 新命令覆盖尚未回复的旧命令，避免回复错位。 */
    Task_Ball_Car_FF_Ready_Pending = 0;

    /* 第一次收到停止帧仅同步状态，不产生一次假的制动前馈。 */
    if(!Task_Ball_Car_Motor_State_Valid){
        Task_Ball_Car_Motor_State_Valid = 1;
        Task_Ball_Car_Motor_Running = IsRunning;
        if(!IsRunning){
            Task_Ball_Car_FF_State = TASK_BALL_CAR_FF_IDLE;
            Feedforward_Applied = Task_Ball_Car_Feedforward_Apply(0.0f);
            if(Feedforward_Applied){
                Task_Ball_Car_FF_Ready_Pending = 1;
                Task_Ball_Car_FF_Ready_Tick = HAL_GetTick();
            }
            return Feedforward_Applied;
        }
    }
    else{
        if(IsRunning == Task_Ball_Car_Motor_Running){
            Feedforward_Applied = Task_Ball_Use_Position_Hold
                               && Task_Ball_Contral_State == TASK_BALL_CONTRAL_RUNNING
                               && BallContral_Get_is_Enable(&BallContral);
            if(Feedforward_Applied){
                Task_Ball_Car_FF_Ready_Pending = 1;
                Task_Ball_Car_FF_Ready_Tick = HAL_GetTick();
            }
            return Feedforward_Applied;
        }
        Task_Ball_Car_Motor_Running = IsRunning;
    }

    Task_Ball_Car_FF_Start_Tick = HAL_GetTick();
    Task_Ball_Car_FF_State = IsRunning ?
                             TASK_BALL_CAR_FF_START :
                             TASK_BALL_CAR_FF_STOP;
    /* 收到双主控事件后立即下发第一拍前馈，不等待下一帧视觉数据。 */
    Feedforward_Applied = Task_Ball_Car_Feedforward_Update();
    if(Feedforward_Applied){
        Task_Ball_Car_FF_Ready_Pending = 1;
        Task_Ball_Car_FF_Ready_Tick = HAL_GetTick();
    }
    return Feedforward_Applied;
}

uint8_t Task_Ball_Contral_Get_Car_Feedforward_Ready(void){
    if(!Task_Ball_Car_FF_Ready_Pending) return 0;

    if(!Task_Ball_Use_Position_Hold
       || Task_Ball_Contral_State != TASK_BALL_CONTRAL_RUNNING
       || !BallContral_Get_is_Enable(&BallContral)){
        Task_Ball_Car_FF_Ready_Pending = 0;
        return 0;
    }

    if((uint32_t)(HAL_GetTick() - Task_Ball_Car_FF_Ready_Tick)
       < BALL_CAR_FF_READY_DELAY_MS){
        return 0;
    }

    Task_Ball_Car_FF_Ready_Pending = 0;
    return 1;
}

float Task_Ball_Contral_Get_Car_Feedforward(void){
    return (float)Task_Ball_Car_FF_Applied;
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

/* Compatibility API used by the current key mapping; not part of the trajectory. */
void Task_Ball_Reset_Zero(void){
    Task_Ball_Trajectory_Cancel();
    Task_Ball_Select_Control(1);
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
