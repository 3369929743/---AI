#include "Task_Ball_Contral.h"
#include "K230.h"
#include "Serial.h"
#include "Ball_Contral.h"
#include "SoftTimer.h"

#define BALL_5CM_OFFSET                 138.0f
#define BALL_TARGET_TOLERANCE           8.0f
#define BALL_STABLE_VELOCITY_LIMIT      30.0f
#define BALL_TARGET_STABLE_FRAME_COUNT  3U

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
static uint8_t Task_Ball_Target_Stable_Frames = 0;

static void Task_Ball_Trajectory_Cancel(void){
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_IDLE;
    Task_Ball_Target_Stable_Frames = 0;
}

static uint8_t Task_Ball_Target_Is_Stable(PID_val Position, PID_val Target){
    PID_val Error = Target - Position;
    PID_val Velocity = BallContral.Ball_Velocity;

    if(Error < 0.0f) Error = -Error;
    if(Velocity < 0.0f) Velocity = -Velocity;
    if(Error <= BALL_TARGET_TOLERANCE && Velocity <= BALL_STABLE_VELOCITY_LIMIT){
        if(Task_Ball_Target_Stable_Frames < BALL_TARGET_STABLE_FRAME_COUNT){
            Task_Ball_Target_Stable_Frames++;
        }
    }
    else{
        Task_Ball_Target_Stable_Frames = 0;
    }

    return Task_Ball_Target_Stable_Frames >= BALL_TARGET_STABLE_FRAME_COUNT;
}

static void Task_Ball_Trajectory_Prepare(PID_val Position){
    if(Task_Ball_Trajectory_State != TASK_BALL_TRAJECTORY_WAIT_ORIGIN) return;

    /* Capture the origin only from a newly decoded vision frame. */
    Task_Ball_Origin_x = Position;
    Task_Ball_Target_Stable_Frames = 0;
    BallContral_Start(&BallContral);
    BallContral_Set_Target(&BallContral, Task_Ball_Origin_x + BALL_5CM_OFFSET);
    Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_PLUS_5CM;
}

static void Task_Ball_Trajectory_Update(PID_val Position){
    switch(Task_Ball_Trajectory_State){
        case TASK_BALL_TRAJECTORY_TO_PLUS_5CM:
            if(Position >= Task_Ball_Origin_x + BALL_5CM_OFFSET - BALL_TARGET_TOLERANCE){
                /* Keep velocity history for a smooth reversal, but discard old integral bias. */
                BallContral_Clear_Integral(&BallContral);
                BallContral_Set_Target(&BallContral, Task_Ball_Origin_x - BALL_5CM_OFFSET);
                Task_Ball_Target_Stable_Frames = 0;
                Task_Ball_Trajectory_State = TASK_BALL_TRAJECTORY_TO_MINUS_5CM;
            }
            break;
        case TASK_BALL_TRAJECTORY_TO_MINUS_5CM:
            if(Task_Ball_Target_Is_Stable(Position, Task_Ball_Origin_x - BALL_5CM_OFFSET)){
                Task_Ball_Target_Stable_Frames = 0;
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
    BallContral_Run(&BallContral, Position);
    Task_Ball_Trajectory_Update(Position);
}

void Task_Ball_Contral_Init(void){
    Serial_Init(&Serial_K230, Serial_3);
    Serial_Init(&Serial_Emm_Ball, Serial_1);
    BallContral_Init(&BallContral, &Serial_K230, &Serial_Emm_Ball, &PID_Ball_Confg);
    K230_Init(&Serial_K230, 0, 0);

    SoftTimer_Init(&SoftTimer_K230, SOFTTIMER_MODE_PERIODIC, 50);

    SoftTimer_Start(&SoftTimer_K230);

    BallContral_Set_Target(&BallContral, 0);
    Task_Ball_Contral_State = TASK_BALL_CONTRAL_IDLE;
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
                SoftTimer_Reset(&SoftTimer_K230);
                Has_New_Frame = 1;
            }
            if(SoftTimer_Trigger(&SoftTimer_K230)){
                Task_Ball_Target_Stable_Frames = 0;
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
    Task_Ball_Target_Stable_Frames = 0;

    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        BallContral_Start(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
    }
}
