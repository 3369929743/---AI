#include "Task_Ball_Contral.h"
#include "K230.h"
#include "Serial.h"
#include "Ball_Contral.h"
#include "SoftTimer.h"

#define BALL_5CM_OFFSET                 138.0f
#define BALL_TARGET_TOLERANCE           4.0f
#define BALL_MINUS_SETTLE_TOLERANCE     3.0f
#define BALL_MINUS_SETTLE_DELTA         3.0f
#define BALL_MINUS_SETTLE_SPEED         25.0f
#define BALL_MINUS_SETTLE_MS            650U
#define BALL_MINUS_SETTLE_FRAMES        8U
#define BALL_MINUS_RELEASE_TOLERANCE    6.0f
#define BALL_MINUS_RELEASE_SPEED        50.0f
#define BALL_MINUS_RELEASE_FRAMES       3U
#define BALL_MINUS_TRIM_WINDOW          24.0f
#define BALL_MINUS_TRIM_SETTLE_MS       250U
#define BALL_MINUS_TRIM_GAIN            0.6f
#define BALL_MINUS_TRIM_STEP_MAX        6.0f
#define BALL_MINUS_TRIM_LIMIT           28.0f
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
static uint32_t Task_Ball_Minus_Settle_Tick = 0;
static uint8_t Task_Ball_Minus_Settle_Frames = 0;
static uint8_t Task_Ball_Minus_Release_Frames = 0;
static PID_val Task_Ball_Plus_Endpoint_x = 0.0f;
static PID_val Task_Ball_Minus_Final_Target_x = 0.0f;
static PID_val Task_Ball_Minus_Target_Trim = 0.0f;

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
}

static void Task_Ball_Minus_Tracking_Reset(void){
    Task_Ball_Minus_Settle_Tick = 0;
    Task_Ball_Minus_Settle_Frames = 0;
    Task_Ball_Minus_Release_Frames = 0;
}

static PID_val Task_Ball_Minus_Base_Target(void){
    return Task_Ball_Origin_x - BALL_5CM_OFFSET;
}

static void Task_Ball_Minus_Final_Reset(void){
    Task_Ball_Plus_Endpoint_x = 0.0f;
    Task_Ball_Minus_Final_Target_x = 0.0f;
    Task_Ball_Minus_Target_Trim = 0.0f;
    Task_Ball_Minus_Tracking_Reset();
}

static void Task_Ball_Minus_Apply_Target(void){
    BallContral_Set_Target(&BallContral,
                           Task_Ball_Minus_Base_Target() + Task_Ball_Minus_Target_Trim);
}

static void Task_Ball_Minus_Begin_Return(PID_val Plus_Endpoint){
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
    Task_Ball_Minus_Final_Reset();
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
                Task_Ball_Minus_Begin_Return(Position);
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
    K230_ResetZero();

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
            if(K230_Error_Update()){
                Task_Ball_Last_Vision_Frame_Tick = HAL_GetTick();
                SoftTimer_Reset(&SoftTimer_K230);
            }
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
    Task_Ball_Minus_Final_Reset();

    if(Task_Ball_Contral_State == TASK_BALL_CONTRAL_IDLE){
        BallContral_Start(&BallContral);
        Task_Ball_Contral_State = TASK_BALL_CONTRAL_RUNNING;
    }
}

void Task_Ball_Reset_Zero(void){
    Task_Ball_Trajectory_Cancel();
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
