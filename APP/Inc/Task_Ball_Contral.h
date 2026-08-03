#ifndef __TASK_BALL_CONTRAL_H__
#define __TASK_BALL_CONTRAL_H__

#include <stdint.h>

typedef enum{
    TASK_BALL_CONTRAL_IDLE = 0,
    TASK_BALL_CONTRAL_RUNNING,
    TASK_BALL_CONTRAL_LOST,
}Task_Ball_Contral_State_e;

typedef struct{
    int32_t Accel_Ay_Filtered;
    int32_t Accel_Ay_Highpass;
    int32_t Pipe_Target_Pulse;
    int32_t Feedforward_x10;
    int32_t Feedback_x10;
    int32_t Combined_x10;
    int32_t Integral_x10;
    int32_t Motor_Actual_Pulse;
    uint32_t Motor_Position_Age_ms;
    uint16_t Feedforward_Blend_x100;
    int16_t Vision_Zero_x;
    int16_t Vision_Error_x;
    int16_t Target_x;
    uint8_t Flags;
}Task_Ball_Debug_Data_t;

void Task_Ball_Contral_Init(void);
void Task_Ball_Contral_Toggle(void);
void Task_Ball_Contral_Loop(void);
void Task_Ball_Contral_Tick(void);
void Task_Ball_Contral_Update_Acceleration(int32_t AccelAy);
float Task_Ball_Contral_Get_IMU_Feedforward(void);
int32_t Task_Ball_Contral_Get_IMU_Accel_Filtered(void);
void Task_Ball_Contral_Get_Debug_Data(Task_Ball_Debug_Data_t *Debug_Data);
void Task_Ball_Contral_Pop_Init(void);
void Task_Ball_Contral_Pop_Ready(void);
void Task_Ball_Contral_Pop_Restore(void);
void Task_Ball_Goto5cm(void);
void Task_Ball_GotoMinus5cm(void);
void Task_Ball_Start_5cm_Sequence(void);
void Task_Ball_Reset_Zero(void);
uint8_t Task_Ball_Get_Control_State(void);
uint8_t Task_Ball_Get_Trajectory_State(void);
uint32_t Task_Ball_Get_Vision_Frame_Age(void);
int16_t Task_Ball_Get_Target_x(void);


#endif
