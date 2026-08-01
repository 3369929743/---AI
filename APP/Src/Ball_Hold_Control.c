#include "Ball_Contral.h"
#include "main.h"

#define BALL_HOLD_VELOCITY_FILTER_TAU_MS 18.0f
#define BALL_FRAME_DT_MIN_MS 2U
#define BALL_FRAME_DT_MAX_MS 200U
#define BALL_INTEGRAL_ERROR_LIMIT 28.0f
#define BALL_INTEGRAL_SPEED_LIMIT 120.0f
#define BALL_PID_OUTPUT_POLARITY (-1.0f)
#define BALL_POSITION_UNITS_PER_CM 27.6f
#define BALL_INTEGRAL_DEADBAND (0.12f * BALL_POSITION_UNITS_PER_CM)
#define BALL_POSITION_LEAD_TIME_S 0.050f
#define BALL_POSITION_LEAD_LIMIT (1.2f * BALL_POSITION_UNITS_PER_CM)
#define BALL_PREDICT_ERROR_ON (0.15f * BALL_POSITION_UNITS_PER_CM)
#define BALL_PREDICT_VELOCITY_ON 40.0f
#define BALL_ESCAPE_VELOCITY_TERM_LIMIT 100.0f
#define BALL_FAST_ERROR_ON (0.35f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_VELOCITY_ON 70.0f
#define BALL_FAST_KP_SCALE 1.85f
#define BALL_FAST_KD_SCALE 2.05f
#define BALL_FAST_MOVING_AWAY_KD_SCALE 1.20f
#define BALL_FAST_OUTPUT_LIMIT 300.0f
#define BALL_FAST_INTEGRAL_DECAY 0.85f
#define BALL_FAST_REARM_ERROR (0.30f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_REARM_VELOCITY 30.0f
#define BALL_FAST_REARM_FRAMES 3U
#define BALL_FAST_RETRIGGER_ERROR (0.15f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_RETRIGGER_VELOCITY 35.0f
#define BALL_FAST_STALL_ERROR (0.65f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_STALL_VELOCITY 20.0f
#define BALL_FAST_ESCAPE_ERROR (0.85f * BALL_POSITION_UNITS_PER_CM)
#define BALL_FAST_EXIT_VELOCITY 60.0f
#define BALL_HOLD_EMERGENCY_ERROR (0.65f * BALL_POSITION_UNITS_PER_CM)
#define BALL_HOLD_EMERGENCY_MIN_OUTPUT 160.0f
#define BALL_APPROACH_ERROR_ON (1.50f * BALL_POSITION_UNITS_PER_CM)
#define BALL_APPROACH_VELOCITY_ON 35.0f
#define BALL_APPROACH_KP_SCALE 0.85f
#define BALL_APPROACH_KD_SCALE 1.10f
#define BALL_APPROACH_D_TERM_LIMIT 90.0f
#define BALL_APPROACH_OUTPUT_LIMIT 140.0f
#define BALL_APPROACH_REVERSE_OUTPUT_LIMIT 75.0f
#define BALL_APPROACH_INTEGRAL_DECAY 0.80f
#define BALL_FINE_ERROR_BAND 8.0f
#define BALL_FINE_VELOCITY_BAND 35.0f
#define BALL_FINE_D_TERM_LIMIT 10.0f
#define BALL_FINE_COMMAND_DEADBAND_PULSE 4
#define BALL_FAST_COMMAND_DEADBAND_PULSE 1
#define BALL_HOLD_OUTPUT_STEP_LIMIT 80
#define BALL_HOLD_REVERSAL_STEP_LIMIT 60
#define BALL_HOLD_QUIET_ENTER_ERROR (0.18f * BALL_POSITION_UNITS_PER_CM)
#define BALL_HOLD_QUIET_RELEASE_ERROR (0.32f * BALL_POSITION_UNITS_PER_CM)
#define BALL_HOLD_QUIET_ENTER_VELOCITY 80.0f
#define BALL_HOLD_QUIET_PIPE_LIMIT 60
#define BALL_HOLD_QUIET_ENTER_FRAMES 4U
#define BALL_POSITION_MOTOR_SPEED 500U
#define BALL_POSITION_MOTOR_ACC 0U
#define BALL_HOLD_MOTOR_SPEED 1100U
#define BALL_HOLD_MOTOR_ACC 240U

static uint8_t Ball_Hold_Quiet_Active = 0;
static uint8_t Ball_Hold_Quiet_Frames = 0;

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
    Ball_Hold_Quiet_Active = 0;
    Ball_Hold_Quiet_Frames = 0;
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
    uint8_t Predict_Response;
    uint8_t Moving_Away;
    uint8_t Moving_Toward;
    uint8_t Renewed_Disturbance;
    uint8_t Recovery_Stalled;
    uint8_t Escaped_From_Target;
    uint8_t Emergency_Response;
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
    Predict_Response = Moving_Away
                    && Abs_Position_Error >= BALL_PREDICT_ERROR_ON
                    && Abs_Velocity >= BALL_PREDICT_VELOCITY_ON;
    Renewed_Disturbance = Moving_Away
                       && Abs_Position_Error >= BALL_FAST_RETRIGGER_ERROR
                       && Abs_Velocity >= BALL_FAST_RETRIGGER_VELOCITY;
    Recovery_Stalled = Abs_Position_Error >= BALL_FAST_STALL_ERROR
                    && Abs_Velocity <= BALL_FAST_STALL_VELOCITY;
    Escaped_From_Target = Moving_Away
                       && Abs_Position_Error >= BALL_FAST_ESCAPE_ERROR;

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
        /* Do not drop the catch gain on a noisy one-frame direction change. */
        if(Moving_Toward
           && Abs_Velocity >= BALL_FAST_EXIT_VELOCITY
           && Abs_Position_Error <= BALL_APPROACH_ERROR_ON){
            BallContral->Fast_Boost_Active = 0;
            BallContral->Fast_Boost_Armed = 0;
        }
    }
    else{
        /*
         * A moving vehicle can disturb the ball again before it has settled
         * inside the small re-arm window.  Re-enter the catch phase when the
         * error starts growing again, or when recovery has visibly stalled.
         * Prediction is still applied only while Moving_Away is true.
         */
        if((BallContral->Fast_Boost_Armed
            && (Abs_Position_Error >= BALL_FAST_ERROR_ON
                || (Moving_Away && Abs_Velocity >= BALL_FAST_VELOCITY_ON)
                || Predict_Response))
           || Renewed_Disturbance
           || Recovery_Stalled
           || Escaped_From_Target){
            BallContral->Fast_Boost_Active = 1;
        }
    }
    Fast_Response = BallContral->Fast_Boost_Active;

    /*
     * Start velocity lead before the larger catch pulse is required.  This
     * keeps inertial prediction responsive without applying it on the return.
     */
    Position_Lead = 0.0f;
    if(Predict_Response || (Fast_Response && Moving_Away)){
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
    Emergency_Response = Fast_Response
                      && Moving_Away
                      && Abs_Error >= BALL_HOLD_EMERGENCY_ERROR;

    Approach_Response = Moving_Toward
                     && Abs_Velocity >= BALL_APPROACH_VELOCITY_ON
                     && Abs_Position_Error <= BALL_APPROACH_ERROR_ON;

    if(Ball_Hold_Quiet_Active){
        /* Preserve the learned static bias without integrating vision noise. */
    }
    else if(Approach_Response){
        PID->ErrorInt *= BALL_APPROACH_INTEGRAL_DECAY;
    }
    else if(Dt_s > 0.0f
       && Abs_Error >= BALL_INTEGRAL_DEADBAND
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

    /*
     * A continuous D term turns frame-to-frame vision jitter into alternating
     * motor commands.  Keep it at zero while settled and on the return path;
     * use a bounded velocity feed-forward only after a real outward escape is
     * visible.  Position hold supplies its own Kd independently from the
     * 5 cm trajectory mode.
     */
    Derivative_Term = 0.0f;
    if(Predict_Response){
        Derivative_Term = Kd * BallContral->Ball_Velocity;
        Derivative_Term = BallContral_Hold_Clamp(Derivative_Term,
                                                 -BALL_ESCAPE_VELOCITY_TERM_LIMIT,
                                                  BALL_ESCAPE_VELOCITY_TERM_LIMIT);
    }

    PID->Output = BALL_PID_OUTPUT_POLARITY
                * (Kp * PID->Cur_Error
                 + PID->Ki * PID->ErrorInt
                 - Derivative_Term);

    /*
     * Treat 0.65 cm predicted error as the last recovery boundary before the
     * 1 cm requirement.  While the ball is still moving away, guarantee a
     * useful pipe angle instead of waiting for the proportional term to grow.
     */
    if(Emergency_Response){
        if(PID->Cur_Error < 0.0f
           && PID->Output < BALL_HOLD_EMERGENCY_MIN_OUTPUT){
            PID->Output = BALL_HOLD_EMERGENCY_MIN_OUTPUT;
        }
        else if(PID->Cur_Error > 0.0f
                && PID->Output > -BALL_HOLD_EMERGENCY_MIN_OUTPUT){
            PID->Output = -BALL_HOLD_EMERGENCY_MIN_OUTPUT;
        }
    }

    /* Limit only the opposite-side braking command; keep outward catch force. */
    if(Approach_Response && (PID->Output * Position_Error) > 0.0f){
        PID->Output = BallContral_Hold_Clamp(
            PID->Output,
            -BALL_APPROACH_REVERSE_OUTPUT_LIMIT,
             BALL_APPROACH_REVERSE_OUTPUT_LIMIT);
    }

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
    uint8_t Moving_Toward;
    uint8_t Escape_Detected;
    uint8_t Reversing_Pipe;
    PID_val Actual_Error;

    if(!BallContral->is_Enable) return;

    Emm = BallContral_Calculate_Inertia_Hold(BallContral, Target);
    Abs_Error = BallContral_Hold_Abs(BallContral->PID_StepMotor.Cur_Error);
    Abs_Velocity = BallContral_Hold_Abs(BallContral->Ball_Velocity);
    Actual_Error = BallContral_Hold_Abs(BallContral->Hold_Target - Target);
    Moving_Toward = ((BallContral->Hold_Target - Target)
                     * BallContral->Ball_Velocity) > 0.0f;
    Escape_Detected = ((BallContral->Hold_Target - Target)
                       * BallContral->Ball_Velocity) < 0.0f
                   && Actual_Error >= BALL_PREDICT_ERROR_ON
                   && Abs_Velocity >= BALL_PREDICT_VELOCITY_ON;
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

    if(Ball_Hold_Quiet_Active){
        if(Actual_Error > BALL_HOLD_QUIET_RELEASE_ERROR
           || Escape_Detected){
            Ball_Hold_Quiet_Active = 0;
            Ball_Hold_Quiet_Frames = 0;
        }
        else{
            /* Keep the pipe still until a real displacement breaks the latch. */
            BallContral->Fast_Boost_Active = 0;
            BallContral->Fast_Boost_Armed = 1;
            BallContral->Ball_Velocity = 0.0f;
            BallContral->PID_StepMotor.Output =
                (PID_val)BallContral->Pipe_Target_Pulse;
            return;
        }
    }
    else if(Actual_Error <= BALL_HOLD_QUIET_ENTER_ERROR
            && Abs_Velocity <= BALL_HOLD_QUIET_ENTER_VELOCITY
            && BallContral->Pipe_Target_Pulse <= BALL_HOLD_QUIET_PIPE_LIMIT
            && BallContral->Pipe_Target_Pulse >= -BALL_HOLD_QUIET_PIPE_LIMIT){
        if(Ball_Hold_Quiet_Frames < BALL_HOLD_QUIET_ENTER_FRAMES){
            Ball_Hold_Quiet_Frames++;
        }
        if(Ball_Hold_Quiet_Frames >= BALL_HOLD_QUIET_ENTER_FRAMES){
            Ball_Hold_Quiet_Active = 1;
            BallContral->Fast_Boost_Active = 0;
            BallContral->Fast_Boost_Armed = 1;
            BallContral->Ball_Velocity = 0.0f;
            BallContral->PID_StepMotor.Output =
                (PID_val)BallContral->Pipe_Target_Pulse;
            return;
        }
    }
    else{
        Ball_Hold_Quiet_Frames = 0;
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

    Reversing_Pipe = ((New_Target_Pulse > 0
                       && BallContral->Pipe_Target_Pulse < 0)
                   || (New_Target_Pulse < 0
                       && BallContral->Pipe_Target_Pulse > 0));
    if(Moving_Toward && Reversing_Pipe){
        Output_Step = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
        if(Output_Step > BALL_HOLD_REVERSAL_STEP_LIMIT){
            New_Target_Pulse = BallContral->Pipe_Target_Pulse
                             + BALL_HOLD_REVERSAL_STEP_LIMIT;
        }
        else if(Output_Step < -BALL_HOLD_REVERSAL_STEP_LIMIT){
            New_Target_Pulse = BallContral->Pipe_Target_Pulse
                             - BALL_HOLD_REVERSAL_STEP_LIMIT;
        }
    }

    Delta_Pulse = New_Target_Pulse - BallContral->Pipe_Target_Pulse;
    if(Delta_Pulse >= Deadband_Pulse || Delta_Pulse <= -Deadband_Pulse){
        Emm_Pos_Run_Quick(&BallContral->Emm_StepMotor, Delta_Pulse);
        BallContral->Pipe_Target_Pulse = New_Target_Pulse;
    }
}
