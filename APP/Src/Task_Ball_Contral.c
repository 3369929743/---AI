#include "Task_Ball_Contral.h"
#include "K230.h"
#include "Serial.h"
#include "Ball_Contral.h"
#include "SoftTimer.h"

/* 5 cm 对应的视觉像素。 */
#define BALL_5CM_OFFSET                  138.0f

/* 正、负 5 cm 分别使用独立的位置死区。 */
#define BALL_PLUS_5CM_DEADBAND           4.0f
#define BALL_MINUS_5CM_DEADBAND          10.0f

/*
 * 正端折返兜底：超过 100 像素后如果在 3 像素范围内停住 200 ms，
 * 即使没有进入正端死区，也立即切换到 -5 cm 目标。
 */
#define BALL_PLUS_MIN_PROGRESS           100.0f
#define BALL_PLUS_STABLE_BAND            3.0f
#define BALL_PLUS_STABLE_MS              200U

/* 视觉数据超时后停机，避免使用旧位置继续控制。 */
#define K230_FRAME_TIMEOUT_MS            250U

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

static PID_Confg_t PID_Ball_Confg = {
    .Kp = 0.55f,
    .Ki = 0.0f,
    .Kd = 0.25f,
    .IntMax = 40.0f,
    .IntMin = -40.0f,
    .OutMax = 220.0f,
    .Alpha = 1.0f,
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

static void Task_Ball_Trajectory_Cancel(void)
{
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

static void Task_Ball_Process_New_Frame(void)
{
    PID_val Position = (PID_val)K230_GetError_x();

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
            if(Task_Ball_Plus5cm_Is_Reached(Position)){
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

    BallContral_Run(&BallContral, Position);
}

void Task_Ball_Contral_Init(void)
{
    Serial_Init(&Serial_K230, Serial_3);
    Serial_Init(&Serial_Emm_Ball, Serial_1);
    BallContral_Init(&BallContral, &Serial_Emm_Ball, &PID_Ball_Confg);
    K230_Init(&Serial_K230, 0, 0);

    SoftTimer_Init(&SoftTimer_K230,
                   SOFTTIMER_MODE_PERIODIC,
                   K230_FRAME_TIMEOUT_MS);
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
            else if(Has_New_Frame){
                Task_Ball_Process_New_Frame();
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

void Task_Ball_Start_5cm_Sequence(void)
{
    /* 丢弃按键触发前缓存的旧帧，下一帧作为轨迹原点。 */
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
