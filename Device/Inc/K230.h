#ifndef __K230_H__
#define __K230_H__

#include <stdint.h>

typedef struct Serial_Struct Serial_t;

void K230_Init(Serial_t *Serial,int16_t Offset_x,int16_t Offset_y);
uint8_t K230_Error_Update(void);
uint8_t K230_GetFlag(void);
void K230_ResetZero(void);
uint8_t K230_IsZeroReady(void);
int16_t K230_GetError_x(void);
int16_t K230_GetError_y(void);

#endif
