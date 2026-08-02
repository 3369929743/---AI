#ifndef __TASK_SERIAL_H__
#define __TASK_SERIAL_H__

#include <stdint.h>

void Task_Serial_Init(void);
void Task_Serial_Loop(void);
uint8_t Task_Serial_Get_Pop_Enable(void);
uint8_t Task_Serial_Get_Car_Motor_State(uint8_t *IsRunning);
uint8_t Task_Serial_Send_Feedforward_Complete(void);

#endif
