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
#define BALL_5CM_OFFSET                 138.0f

/* 到达正向 5 cm 目标的像素死区：误差不超过 4 像素即判定到位。 */
#define BALL_TARGET_TOLERANCE           8.0f

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
#define K230_FRAME_TIMEOUT_MS           250U

/* 正向 5 cm 运动超时时间：超过 2 s 后自动切换到负向 5 cm。 */
#define BALL_PLUS_5CM_TIMEOUT_MS        2000U

typedef enum{
    TASK_BALL_TRAJECTORY_IDLE = 0,
    TASK_BALL_TRAJECTORY_WAIT_ORIGIN,
    TASK_BALL_TRAJECTORY_TO_PLUS_5CM,
    TASK_BALL_TRAJECTORY_TO_MINUS_5CM,
}Task_Ball_Trajectory_State_e;

static Serial_t Serial_K230;
static Serial_t Serial_Emm_Ball;
static SoftTimer_t SoftTimer_K230;

static BallContral_t BallContral;

/* 5 cm position trajectory: keep its original damping parameters. */
static PID_Confg_t PID_Ball_5cm_Confg = {
    .Kp = 0.9,
    .Ki = 0.7,
    .Kd = 0.45,
    .IntMax = 40,
    .IntMin = -40,
    .OutMax = 220,
    .Alpha = 1.0
};

/* Position hold: filtered damping is active during escape and return. */
static PID_Confg_t PID_Ball_Hold_Confg = {
    .Kp = 2.5,
    .Ki = 0.45,
    .Kd = 0.45,
    .IntMax = 60,
    .IntMin = -60,
    .OutMax = 220,
    .Alpha = 1.0
};

static Task_Ball_Contral_State_e Task_Ball_Contral_State =
    TASK_BALL_CONTRAL_IDLE;
static Task_Ball_Trajectory_State_e Task_Ball_Trajectory_State =
    TASK_BALL_TRAJECTORY_IDLE;
static PID_val Task_Ball_Origin_x = 0.0f;
static PID_val Task_Ball_Plus_Target_x = 0.0f;
static PID_val Task_Ball_Minus_Target_x = 0.0f;
static PID_val Task_Ball_Plus_Stable_Anchor_x = 0.0f;
static uint32_t Task_Ball_Plus_Stable_Tick = 0U;
static uint8_t Task_Ball_Plus_Stable_Tracking = 0U;

static void Task_Ball_Set_Target(PID_val Target, PID_val Deadband)
{
    BallContral_Set_Target(&BallContral, Target, Deadband);
}

static void Task_Ball_Plus_Stable_Reset(void)
{
    Task_Ball_Plus_Stable_Anchor_x = 0.0f;
    Task_Ball_Plus_Stable_Tick = 0U;
    Task_Ball_Plus_Stable_Tracking = 0U;
}

static void Task_Ball_Trajectory_Cancel(void){
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_IDLE;
    Task_Ball_Plus_Stable_Reset();
}

static uint8_t Task_Ball_Plus5cm_Is_Reached(PID_val Position)
{
    PID_val Progress = Position - Task_Ball_Origin_x;
    PID_val Anchor_Error;

    if(Position >= Task_Ball_Plus_Target_x - BALL_PLUS_5CM_DEADBAND){
        Task_Ball_Plus_Stable_Reset();
        return 1U;
    }

    if(Progress < BALL_PLUS_MIN_PROGRESS){
        Task_Ball_Plus_Stable_Reset();
        return 0U;
    }

    if(!Task_Ball_Plus_Stable_Tracking){
        Task_Ball_Plus_Stable_Anchor_x = Position;
        Task_Ball_Plus_Stable_Tick = HAL_GetTick();
        Task_Ball_Plus_Stable_Tracking = 1U;
        return 0U;
    }

    Anchor_Error = Position - Task_Ball_Plus_Stable_Anchor_x;
    if(Anchor_Error < 0.0f) Anchor_Error = -Anchor_Error;
    if(Anchor_Error > BALL_PLUS_STABLE_BAND){
        Task_Ball_Plus_Stable_Anchor_x = Position;
        Task_Ball_Plus_Stable_Tick = HAL_GetTick();
        return 0U;
    }

    if((uint32_t)(HAL_GetTick() - Task_Ball_Plus_Stable_Tick)
       >= BALL_PLUS_STABLE_MS){
        Task_Ball_Plus_Stable_Reset();
        return 1U;
    }

    return 0U;
}

static void Task_Ball_Trajectory_Prepare(PID_val Position){
    if(Task_Ball_Trajectory_State != TASK_BALL_TRAJECTORY_WAIT_ORIGIN) return;

    /* Capture the origin only from a newly decoded vision frame. */
    Task_Ball_Origin_x = Position;
    Task_Ball_Endpoint_Tracking_Reset();
    Task_Ball_Minus_Final_Reset();
    BallContral_Start(&BallContral);
    BallContral_Set_Target(&BallContral, Task_Ball_Origin_x + BALL_5CM_OFFSET);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_PLUS_5CM;
}

static void Task_Ball_Trajectory_Update(PID_val Position){
    switch(Task_Ball_Trajectory_State){
        case TASK_BALL_TRAJECTORY_WAIT_ORIGIN:
            /* 启动后的第一帧只用于记录本次轨迹的原点。 */
            Task_Ball_Origin_x = Position;
            Task_Ball_Plus_Target_x = Task_Ball_Origin_x
                                    + BALL_5CM_OFFSET;
            Task_Ball_Minus_Target_x = Task_Ball_Origin_x
                                     - BALL_5CM_OFFSET;
            Task_Ball_Plus_Stable_Reset();
            Task_Ball_Set_Target(Task_Ball_Plus_Target_x,
                                 BALL_PLUS_5CM_DEADBAND);
            Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_PLUS_5CM;
            break;

        case TASK_BALL_TRAJECTORY_TO_PLUS_5CM:
            if(Task_Ball_Target_Is_Reached(Position,
                                           Task_Ball_Origin_x + BALL_5CM_OFFSET,
                                           1)){
                /* Keep velocity history for a smooth reversal, but discard old integral bias. */
                BallContral_Clear_Integral(&BallContral);
                Task_Ball_Set_Target(Task_Ball_Minus_Target_x,
                                     BALL_MINUS_5CM_DEADBAND);
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
            }
            break;

        case TASK_BALL_TRAJECTORY_TO_MINUS_5CM:
        case TASK_BALL_TRAJECTORY_IDLE:
        default:
            break;
    }
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

void Task_Ball_Contral_Init(void)
{
    Serial_Init(&Serial_K230, Serial_3);
    Serial_Init(&Serial_Emm_Ball, Serial_1);
    BallContral_Init(&BallContral, &Serial_Emm_Ball, &PID_Ball_Confg);
    K230_Init(&Serial_K230, 0, 0);

    SoftTimer_Init(&SoftTimer_K230, SOFTTIMER_MODE_PERIODIC, K230_FRAME_TIMEOUT_MS);

    SoftTimer_Start(&SoftTimer_K230);

    Task_Ball_Set_Target(0.0f, 0.0f);
    Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
    Task_Ball_Trajectory_Cancel();
}

void Task_Ball_Contral_Toggle(void)
{
    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        Task_Ball_Trajectory_Cancel();
        K230_ResetZero();
        Task_Ball_Set_Target(0.0f, 0.0f);
        SoftTimer_Reset(&SoftTimer_K230);
        BallContral_Start(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
    }
    else{
        Task_Ball_Trajectory_Cancel();
        BallContral_Stop(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
    }
}

void Task_Ball_Contral_Loop(void)
{
    uint8_t Has_New_Frame = 0U;

    switch(Task_Ball_Contral_State){
        case TASK_BALL_CONTRAL_IDLE:
            break;

        case TASK_BALL_CONTRAL_RUNNING:
            if(K230_Error_Update()){
                SoftTimer_Reset(&SoftTimer_K230);
                Has_New_Frame = 1U;
            }

            if(SoftTimer_Trigger(&SoftTimer_K230)){
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
            if(K230_Error_Update()){
                SoftTimer_Reset(&SoftTimer_K230);
                BallContral_Start(&BallContral);
                Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
                Task_Ball_Process_New_Frame();
            }
            break;
    }
}

void Task_Ball_Contral_Tick(void)
{
    SoftTimer_Update(&SoftTimer_K230);
    SoftTimer_Update(&SoftTimer_Ball_Plus_5cm);
}

void Task_Ball_Contral_Pop_Init(void)
{
    Ball_Contral_Emm_Quick_Init(&BallContral);
}

void Task_Ball_Contral_Pop_Ready(void)
{
    Ball_Contral_Pop_Run(&BallContral, -100);
}

void Task_Ball_Contral_Pop_Restore(void)
{
    Ball_Contral_Pop_Run(&BallContral, 100);
}

void Task_Ball_Goto5cm(void)
{
    Task_Ball_Trajectory_Cancel();
    Task_Ball_Set_Target(BALL_5CM_OFFSET, BALL_PLUS_5CM_DEADBAND);
}

void Task_Ball_GotoMinus5cm(void)
{
    Task_Ball_Trajectory_Cancel();
    Task_Ball_Set_Target(-BALL_5CM_OFFSET, BALL_MINUS_5CM_DEADBAND);
}

void Task_Ball_Start_5cm_Sequence(void){
    /* Discard a frame buffered before the trigger, then wait for a fresh origin. */
    Task_Ball_Select_Control(0);
    (void)K230_GetFlag();
    SoftTimer_Reset(&SoftTimer_K230);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_WAIT_ORIGIN;
    Task_Ball_Plus_Stable_Reset();

    if(Task_Ball_Contral_State != TASK_BALL_CONTRAL_RUNNING){
        BallContral_Start(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
    }
}

void Task_Ball_Reset_Zero(void)
{
    Task_Ball_Trajectory_Cancel();
    K230_ResetZero();
    SoftTimer_Reset(&SoftTimer_K230);
    BallContral_Clear_Integral(&BallContral);
    Task_Ball_Set_Target(0.0f, 0.0f);
}
