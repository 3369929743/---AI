#ifndef __TASK_BALL_CONTRAL_H__
#define __TASK_BALL_CONTRAL_H__

#include <stdint.h>

typedef enum{
    TASK_BALL_CONTRAL_IDLE = 0,
    TASK_BALL_CONTRAL_RUNNING,
    TASK_BALL_CONTRAL_LOST,
}Task_Ball_Contral_State_e;

void Task_Ball_Contral_Init(void);
void Task_Ball_Contral_Toggle(void);
void Task_Ball_Contral_Loop(void);
void Task_Ball_Contral_Tick(void);
void Task_Ball_Contral_Pop_Init(void);
void Task_Ball_Contral_Pop_Ready(void);
void Task_Ball_Contral_Pop_Restore(void);
void Task_Ball_Goto5cm(void);
void Task_Ball_GotoMinus5cm(void);
void Task_Ball_Start_5cm_Sequence(void);
uint8_t Task_Ball_Get_Control_State(void);
uint8_t Task_Ball_Get_Trajectory_State(void);
uint32_t Task_Ball_Get_Vision_Frame_Age(void);
int16_t Task_Ball_Get_Target_x(void);


#endif
