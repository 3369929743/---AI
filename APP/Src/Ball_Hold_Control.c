#include "Ball_Contral.h"
#include "main.h"

#define BALL_HOLD_VELOCITY_FILTER_TAU_MS 30.0f
#define BALL_FRAME_DT_MIN_MS 2U
#define BALL_FRAME_DT_MAX_MS 200U
#define BALL_INTEGRAL_ERROR_LIMIT 28.0f
#define BALL_INTEGRAL_SPEED_LIMIT 120.0f
#define BALL_PID_OUTPUT_POLARITY (-1.0f)
#define BALL_POSITION_UNITS_PER_CM 27.6f
#define BALL_POSITION_LEAD_TIME_S 0.035f
#define BALL_POSITION_LEAD_LIMIT (1.0f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_ERROR_ON (0.65f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_VELOCITY_ON 150.0f
#define BALL_FAST_KP_SCALE 1.55f
#define BALL_FAST_KD_SCALE 1.75f
#define BALL_FAST_MOVING_AWAY_KD_SCALE 1.20f
#define BALL_FAST_OUTPUT_LIMIT 300.0f
#define BALL_FAST_INTEGRAL_DECAY 0.85f
#define BALL_FAST_REARM_ERROR (0.30f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_REARM_VELOCITY 30.0f
#define BALL_FAST_REARM_FRAMES 3U
#define BALL_APPROACH_ERROR_ON (1.50f * BALL_POSITION_UNITS_PER_CM)
#define BALL_APPROACH_VELOCITY_ON 35.0f
#define BALL_APPROACH_KP_SCALE 0.85f
#define BALL_APPROACH_KD_SCALE 1.10f
#define BALL_APPROACH_D_TERM_LIMIT 90.0f
#define BALL_APPROACH_OUTPUT_LIMIT 140.0f
#define BALL_APPROACH_INTEGRAL_DECAY 0.80f
#define BALL_FINE_ERROR_BAND 8.0f
#define BALL_FINE_VELOCITY_BAND 35.0f
#define BALL_FINE_COMMAND_DEADBAND_PULSE 1
#define BALL_FAST_COMMAND_DEADBAND_PULSE 1
#define BALL_HOLD_OUTPUT_STEP_LIMIT 80
#define BALL_POSITION_MOTOR_SPEED 500U
#define BALL_POSITION_MOTOR_ACC 0U
#define BALL_HOLD_MOTOR_SPEED 900U
#define BALL_HOLD_MOTOR_ACC 200U

static PID_val BallContral_Hold_Abs(PID_val Value)
{
    return (Value < 0.0f) ? -Value : Value;
}

static PID_val BallContral_Hold_Clamp(PID_val Value, PID_val Min, PID_val Max)
{
    if(Value > Max) return Max;
    if(Value < Min) return Min;
    return Value;
}

static void BallContral_Hold_Reset_State(BallContral_t *BallContral)
{
    PID_Clear(&BallContral->PID_StepMotor);
    BallContral->Ball_Velocity = 0.0f;
    BallContral->Has_Ball_History = 0;
    BallContral->Fast_Boost_Armed = 1;
    BallContral->Fast_Boost_Active = 0;
    BallContral->Fast_Boost_Settle_Frames = 0;
}

void BallContral_Position_Mode_Init(BallContral_t *BallContral)
{
    Emm_SetSpeed(&BallContral->Emm_StepMotor, BALL_POSITION_MOTOR_SPEED);
    Emm_SetAcc(&BallContral->Emm_StepMotor, BALL_POSITION_MOTOR_ACC);
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
    BallContral_Hold_Reset_State(BallContral);
}

void BallContral_Hold_Mode_Init(BallContral_t *BallContral)
{
    Emm_SetSpeed(&BallContral->Emm_StepMotor, BALL_HOLD_MOTOR_SPEED);
    Emm_SetAcc(&BallContral->Emm_StepMotor, BALL_HOLD_MOTOR_ACC);
    Emm_Pos_Control_Quick_Init(&BallContral->Emm_StepMotor);
    BallContral_Hold_Reset_State(BallContral);
    BallContral->Hold_Target = 0.0f;
    PID_Set_Target(&BallContral->PID_StepMotor, BallContral->Hold_Target);
}

static PID_val BallContral_Calculate_Inertia_Hold(BallContral_t *BallContral,
                                                   PID_val Actual)
{
    PID_t *PID = &BallContral->PID_StepMotor;
    uint32_t Now = HAL_GetTick();
    uint32_t Dt_ms = Now - BallContral->Last_Frame_Tick;
    PID_val Dt_s = 0.0f;
    PID_val Abs_Error;
    PID_val Abs_Velocity;
    PID_val Kp;
    PID_val Kd;
    PID_val OutputLimit;
    PID_val Position_Lead;
    PID_val Position_Error;
    PID_val Abs_Position_Error;
    PID_val Derivative_Term;
    uint8_t Fast_Response;
    uint8_t Moving_Away;
    uint8_t Moving_Toward;
    uint8_t Approach_Response;

    if(!BallContral->Has_Ball_History){
        BallContral->Ball_Position_Pre = Actual;
        BallContral->Ball_Velocity = 0.0f;
        BallContral->Has_Ball_History = 1;
        BallContral->Fast_Boost_Armed = 1;
        BallContral->Fast_Boost_Active = 0;
        BallContral->Fast_Boost_Settle_Frames = 0;
    }
    else if(Dt_ms >= BALL_FRAME_DT_MIN_MS && Dt_ms <= BALL_FRAME_DT_MAX_MS){
        PID_val Alpha = (PID_val)Dt_ms
                      / (BALL_HOLD_VELOCITY_FILTER_TAU_MS + (PID_val)Dt_ms);
        PID_val Raw_Velocity = (Actual - BallContral->Ball_Position_Pre)
                             * 1000.0f / (PID_val)Dt_ms;

        BallContral->Ball_Velocity += Alpha
                                    * (Raw_Velocity - BallContral->Ball_Velocity);
        Dt_s = (PID_val)Dt_ms * 0.001f;
    }
    else{
        BallContral->Ball_Velocity = 0.0f;
    }

    BallContral->Ball_Position_Pre = Actual;
    BallContral->Last_Frame_Tick = Now;

    Abs_Velocity = BallContral_Hold_Abs(BallContral->Ball_Velocity);
    Position_Error = BallContral->Hold_Target - Actual;
    Abs_Position_Error = BallContral_Hold_Abs(Position_Error);
    Moving_Away = (Position_Error * BallContral->Ball_Velocity) < 0.0f;
    Moving_Toward = (Position_Error * BallContral->Ball_Velocity) > 0.0f;

    if(Abs_Position_Error <= BALL_FAST_REARM_ERROR
       && Abs_Velocity <= BALL_FAST_REARM_VELOCITY){
        if(BallContral->Fast_Boost_Settle_Frames < BALL_FAST_REARM_FRAMES){
            BallContral->Fast_Boost_Settle_Frames++;
        }
        if(BallContral->Fast_Boost_Settle_Frames >= BALL_FAST_REARM_FRAMES){
            BallContral->Fast_Boost_Armed = 1;
            BallContral->Fast_Boost_Active = 0;
        }
    }
    else{
        BallContral->Fast_Boost_Settle_Frames = 0;
    }

    if(BallContral->Fast_Boost_Active){
        if(Moving_Toward && Abs_Position_Error <= BALL_APPROACH_ERROR_ON){
            BallContral->Fast_Boost_Active = 0;
            BallContral->Fast_Boost_Armed = 0;
        }
    }
    else if(BallContral->Fast_Boost_Armed
            && (Abs_Position_Error >= BALL_FAST_ERROR_ON
                || (Moving_Away && Abs_Velocity >= BALL_FAST_VELOCITY_ON))){
        BallContral->Fast_Boost_Active = 1;
    }
    Fast_Response = BallContral->Fast_Boost_Active;

    /* Predict only while the disturbance is carrying the ball away. */
    Position_Lead = 0.0f;
    if(Fast_Response && Moving_Away){
        Position_Lead = BallContral->Ball_Velocity * BALL_POSITION_LEAD_TIME_S;
        Position_Lead = BallContral_Hold_Clamp(Position_Lead,
                                               -BALL_POSITION_LEAD_LIMIT,
                                                BALL_POSITION_LEAD_LIMIT);
    }

    PID->Target = BallContral->Hold_Target;
    PID->Actual = Actual + Position_Lead;
    PID->Pre_Error = PID->Cur_Error;
    PID->Cur_Error = PID->Target - PID->Actual;
    PID->Error_Rate_Filter = -BallContral->Ball_Velocity;
    Abs_Error = BallContral_Hold_Abs(PID->Cur_Error);

    Approach_Response = Moving_Toward
                     && Abs_Velocity >= BALL_APPROACH_VELOCITY_ON
                     && Abs_Position_Error <= BALL_APPROACH_ERROR_ON;

    if(Approach_Response){
        PID->ErrorInt *= BALL_APPROACH_INTEGRAL_DECAY;
    }
    else if(Dt_s > 0.0f
       && Abs_Error <= BALL_INTEGRAL_ERROR_LIMIT
       && Abs_Velocity <= BALL_INTEGRAL_SPEED_LIMIT){
        PID->ErrorInt += PID->Cur_Error * Dt_s;
    }
    else if(Fast_Response){
        PID->ErrorInt *= BALL_FAST_INTEGRAL_DECAY;
    }
    if(PID->ErrorInt > PID->IntMax) PID->ErrorInt = PID->IntMax;
    else if(PID->ErrorInt < PID->IntMin) PID->ErrorInt = PID->IntMin;

    Kp = PID->Kp;
    Kd = PID->Kd;
    OutputLimit = PID->OutMax;
    if(Approach_Response){
        Kp *= BALL_APPROACH_KP_SCALE;
        Kd *= BALL_APPROACH_KD_SCALE;
        OutputLimit = BALL_APPROACH_OUTPUT_LIMIT;
    }
    else if(Fast_Response){
        Kp *= BALL_FAST_KP_SCALE;
        Kd *= BALL_FAST_KD_SCALE;
        if(Moving_Away){
            Kd *= BALL_FAST_MOVING_AWAY_KD_SCALE;
        }
        OutputLimit = BALL_FAST_OUTPUT_LIMIT;
    }

    Derivative_Term = Kd * BallContral->Ball_Velocity;
    if(Approach_Response){
        Derivative_Term = BallContral_Hold_Clamp(Derivative_Term,
                                                 -BALL_APPROACH_D_TERM_LIMIT,
                                                  BALL_APPROACH_D_TERM_LIMIT);
    }

    PID->Output = BALL_PID_OUTPUT_POLARITY
                * (Kp * PID->Cur_Error
                 + PID->Ki * PID->ErrorInt
                 - Derivative_Term);

    if(PID->Output > OutputLimit) PID->Output = OutputLimit;
    else if(PID->Output < -OutputLimit) PID->Output = -OutputLimit;

    return PID->Output;
}

void BallContral_Run_Inertia_Hold(BallContral_t *BallContral, PID_val Target)
{
    PID_val Emm;
    PID_val Abs_Error;
    PID_val Abs_Velocity;
    int32_t New_Target_Pulse;
    int32_t Delta_Pulse;
    int32_t Deadband_Pulse;
    int32_t Output_Step;

    (void)Target;
    if(!BallContral->is_Enable) return;

    Emm = BallContral_Calculate_Inertia_Hold(BallContral, Target);
    Abs_Error = BallContral_Hold_Abs(BallContral->PID_StepMotor.Cur_Error);
    Abs_Velocity = BallContral_Hold_Abs(BallContral->Ball_Velocity);
    Deadband_Pulse = (Abs_Error <= BALL_FINE_ERROR_BAND
                      && Abs_Velocity <= BALL_FINE_VELOCITY_BAND) ?
                     BALL_FINE_COMMAND_DEADBAND_PULSE :
                     BALL_FAST_COMMAND_DEADBAND_PULSE;

    if(Emm >= 0){
        New_Target_Pulse = (int32_t)(Emm + 0.5f);
    }
    else{
        New_Target_Pulse = (int32_t)(Emm - 0.5f);
    }

    /* After the catch pulse, prevent frame-to-frame full-angle reversals. */
    if(!BallContral->Fast_Boost_Active){
        Output_Step = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
        if(Output_Step > BALL_HOLD_OUTPUT_STEP_LIMIT){
            New_Target_Pulse = BallContral->Pipe_Target_Pulse
                             + BALL_HOLD_OUTPUT_STEP_LIMIT;
        }
        else if(Output_Step < -BALL_HOLD_OUTPUT_STEP_LIMIT){
            New_Target_Pulse = BallContral->Pipe_Target_Pulse
                             - BALL_HOLD_OUTPUT_STEP_LIMIT;
        }
    }

    Delta_Pulse = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Delta_Pulse >= Deadband_Pulse || Delta_Pulse <= -Deadband_Pulse){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Delta_Pulse);
        BallContral->Pipe_Target_Pulse = New_Target_Pulse;
    }
}
