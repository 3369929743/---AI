#ifndef __BALL_IMPACT_CONFIG_H__
#define __BALL_IMPACT_CONFIG_H__

#include <stdint.h>

#define BALL_IMPACT_CONFIG_STEP_PULSE       5
#define BALL_IMPACT_CONFIG_MIN_PULSE        0U
#define BALL_IMPACT_CONFIG_MAX_PULSE       60U

void BallImpactConfig_Init(void);
void BallImpactConfig_Adjust_Start_Lower(int16_t DeltaPulse);
void BallImpactConfig_Adjust_Stop_Raise(int16_t DeltaPulse);
uint8_t BallImpactConfig_Save(void);

uint16_t BallImpactConfig_Get_Start_Lower(void);
uint16_t BallImpactConfig_Get_Stop_Raise(void);
uint8_t BallImpactConfig_Is_Dirty(void);
uint8_t BallImpactConfig_Has_Save_Error(void);

#endif
