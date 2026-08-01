#ifndef __JY61P_H__
#define __JY61P_H__

#include <stdint.h>

typedef struct Serial_Struct Serial_t;

typedef struct {
    int16_t Ax;
    int16_t Ay;
    int16_t Az;
    int16_t Temp;
} JY61P_Accel_t;

void JY61P_Init(Serial_t *Serial);
uint8_t JY61P_Accel_Update(void);
void JY61P_Get_Accel(JY61P_Accel_t *Accel);

#endif