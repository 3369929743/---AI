#include "Task_Ball_Contral.h"
#include "K230.h"
#include "Serial.h"
#include "Ball_Contral.h"
#include "SoftTimer.h"

#define BALL_5CM_OFFSET                 138.0f
#define BALL_TARGET_TOLERANCE           4.0f
#define BALL_ENDPOINT_MIN_PROGRESS      100.0f
#define BALL_ENDPOINT_STABLE_BAND       3.0f
#define BALL_ENDPOINT_STABLE_MS         200U
#define K230_FRAME_TIMEOUT_MS           250U

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

static BallContral_t BallContral;

static PID_Confg_t PID_Ball_Confg = {
    .Kp = 0.9,
    .Ki = 0.7,
    .Kd = 0.45,
    .IntMax = 40,
    .IntMin = -40,
    .OutMax = 220
};

static Task_Ball_Contral_State_e Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
static Task_Ball_Trajectory_State_e Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_IDLE;
static PID_val Task_Ball_Origin_x = 0.0f;
static uint32_t Task_Ball_Last_Vision_Frame_Tick = 0;
static PID_val Task_Ball_Endpoint_Anchor_x = 0.0f;
static uint32_t Task_Ball_Endpoint_Stable_Tick = 0;
static uint8_t Task_Ball_Endpoint_Tracking = 0;

static void Task_Ball_Endpoint_Tracking_Reset(void){
    Task_Ball_Endpoint_Anchor_x = 0.0f;
    Task_Ball_Endpoint_Stable_Tick = 0;
    Task_Ball_Endpoint_Tracking = 0;
}

static void Task_Ball_Trajectory_Cancel(void){
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_IDLE;
    Task_Ball_Endpoint_Tracking_Reset();
}

static uint8_t Task_Ball_Target_Is_Reached(PID_val Position, PID_val Target,
                                           int8_t Direction){
    PID_val Progress;
    PID_val Anchor_Error;

    if(Direction > 0){
        if(Position >= Target - BALL_TARGET_TOLERANCE){
            Task_Ball_Endpoint_Tracking_Reset();
            return 1;
        }
        Progress = Position - Task_Ball_Origin_x;
    }
    else{
        if(Position <= Target + BALL_TARGET_TOLERANCE){
            Task_Ball_Endpoint_Tracking_Reset();
            return 1;
        }
        Progress = Task_Ball_Origin_x - Position;
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
    BallContral_Start(&BallContral);
    BallContral_Set_Target(&BallContral, Task_Ball_Origin_x + BALL_5CM_OFFSET);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_PLUS_5CM;
}

static void Task_Ball_Trajectory_Update(PID_val Position){
    switch(Task_Ball_Trajectory_State){
        case TASK_BALL_TRAJECTORY_TO_PLUS_5CM:
            if(Task_Ball_Target_Is_Reached(Position,
                                           Task_Ball_Origin_x + BALL_5CM_OFFSET,
                                           1)){
                /* Keep velocity history for a smooth reversal, but discard old integral bias. */
                BallContral_Clear_Integral(&BallContral);
                BallContral_Set_Target(&BallContral, Task_Ball_Origin_x - BALL_5CM_OFFSET);
                Task_Ball_Endpoint_Tracking_Reset();
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
            }
            break;
        case TASK_BALL_TRAJECTORY_TO_MINUS_5CM:
            if(Task_Ball_Target_Is_Reached(Position,
                                           Task_Ball_Origin_x - BALL_5CM_OFFSET,
                                           -1)){
                Task_Ball_Endpoint_Tracking_Reset();
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM;
            }
            break;
        case TASK_BALL_TRAJECTORY_IDLE:
        case TASK_BALL_TRAJECTORY_WAIT_ORIGIN:
        case TASK_BALL_TRAJECTORY_HOLD_MINUS_5CM:
        default:
            break;
    }
}

static void Task_Ball_Process_New_Frame(void){
    PID_val Position = (PID_val)K230_GetError_x();

    Task_Ball_Trajectory_Prepare(Position);
    /* Switch the target before calculating this frame's PID output. */
    Task_Ball_Trajectory_Update(Position);
    BallContral_Run(&BallContral, Position);
}

void Task_Ball_Contral_Init(void){
    Serial_Init(&Serial_K230, Serial_3);
    Serial_Init(&Serial_Emm_Ball, Serial_1);
    BallContral_Init(&BallContral, &Serial_K230, &Serial_Emm_Ball, &PID_Ball_Confg);
    K230_Init(&Serial_K230, 0, 0);

    SoftTimer_Init(&SoftTimer_K230, SOFTTIMER_MODE_PERIODIC, K230_FRAME_TIMEOUT_MS);

    SoftTimer_Start(&SoftTimer_K230);

    BallContral_Set_Target(&BallContral, 0);
    Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
    Task_Ball_Last_Vision_Frame_Tick = HAL_GetTick();
    Task_Ball_Trajectory_Cancel();
}

void Task_Ball_Contral_Toggle(void){
    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        Task_Ball_Trajectory_Cancel();
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
                BallContral_Stop(&BallContral);
                Task_Ball_Contral_State = TASK_BALL_CONTRAL_LOST;
            }
            else if(Has_New_Frame){
                Task_Ball_Process_New_Frame();
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

void Task_Ball_Contral_Tick(void){
    SoftTimer_Update(&SoftTimer_K230);
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
    BallContral_Set_Target(&BallContral, BALL_5CM_OFFSET);
}

void Task_Ball_GotoMinus5cm(void){
    Task_Ball_Trajectory_Cancel();
    BallContral_Set_Target(&BallContral, -BALL_5CM_OFFSET);
}

void Task_Ball_Start_5cm_Sequence(void){
    /* Discard a frame buffered before the trigger, then wait for a fresh origin. */
    (void)K230_GetFlag();
    SoftTimer_Reset(&SoftTimer_K230);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_WAIT_ORIGIN;
    Task_Ball_Endpoint_Tracking_Reset();

    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        BallContral_Start(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
    }
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
