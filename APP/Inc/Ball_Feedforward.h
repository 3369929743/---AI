#ifndef __BALL_FEEDFORWARD_H__
#define __BALL_FEEDFORWARD_H__

#include <stdint.h>
#include "PID.h"

typedef enum{
    BALL_FEEDFORWARD_IDLE = 0,
    BALL_FEEDFORWARD_ACTIVE,
    BALL_FEEDFORWARD_RETURNING,
}BallFeedforward_State_e;

typedef struct{
    BallFeedforward_State_e State;
    PID_val Raw_Acceleration;
    PID_val Filtered_Acceleration;
    PID_val Baseline_Pulse;
    PID_val Baseline_Actual_Pulse;
    PID_val Target_Offset_Pulse;
    PID_val Position_Blend;
    uint32_t Last_Sample_Tick;
    uint32_t Confirm_Tick;
    uint32_t Return_Tick;
    uint8_t Has_Sample;
    uint8_t Enter_Confirming;
    uint8_t Exit_Confirming;
}BallFeedforward_t;

void BallFeedforward_Init(BallFeedforward_t *Feedforward);
void BallFeedforward_Reset_Control(BallFeedforward_t *Feedforward,
                                   int32_t Current_Pipe_Pulse);
void BallFeedforward_Update(BallFeedforward_t *Feedforward,
                            int32_t Acceleration,
                            PID_val Position_Error,
                            int32_t Current_Pipe_Pulse,
                            PID_val Actual_Pipe_Pulse,
                            uint8_t Position_Feedback_Valid,
                            uint8_t Enable,
                            uint32_t Now);
uint8_t BallFeedforward_Check_Timeout(BallFeedforward_t *Feedforward,
                                      int32_t Current_Pipe_Pulse,
                                      uint32_t Now);

#endif
