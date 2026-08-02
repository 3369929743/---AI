#ifndef __BALL_CONTRAL_H__
#define __BALL_CONTRAL_H__

#include "Emm_v5.h"
#include "PID.h"

typedef struct BallContral_Struct{
    uint8_t is_Enable;
    Emm_t Emm_StepMotor;
    PID_t PID_StepMotor;

    int32_t Pipe_Target_Pulse;
    PID_val Position_Deadband;

    PID_val Ball_Position_Pre;
    PID_val Ball_Velocity;
    uint32_t Last_Frame_Tick;
    uint8_t Has_Ball_History;
}BallContral_t;

void BallContral_Init(BallContral_t *BallContral,
                      Serial_t *Serial_Emm,
                      PID_Confg_t *PID_Confg);
void BallContral_Set_Target(BallContral_t *BallContral,
                            PID_val Target,
                            PID_val Deadband);
void BallContral_Clear_Integral(BallContral_t *BallContral);
void BallContral_Run(BallContral_t *BallContral, PID_val Actual);
uint8_t BallContral_Get_is_Enable(BallContral_t *BallContral);
void BallContral_Start(BallContral_t *BallContral);
void BallContral_Stop(BallContral_t *BallContral);
void Ball_Contral_Emm_Quick_Init(BallContral_t *BallContral);
void Ball_Contral_Pop_Run(BallContral_t *BallContral, int32_t Pulse);

#endif
