#ifndef __EMM_V5_H__
#define __EMM_V5_H__

#include "stdbool.h"
#include <stdint.h>

typedef struct Serial_Struct Serial_t;

typedef struct {
    Serial_t *Serial;     // 串口结构体指针
    uint8_t Addr;       // 地址
    uint8_t Dir;        // 方向
    uint16_t Vel;       // 速度
    uint8_t Acc;        // 加速度
    int32_t Pul;       // 脉冲数
    uint8_t raF;           // 相对/绝对标志，0：相对上一输入目标位置进行相对位置运动, 1:相对坐标零点进行绝对位置运动, 2:相对当前实时位置进行相对位置运动
    bool snF;           // 多机同步运动标志，0：不同步，1：同步

    /* 0x36 实时位置回读状态；原始位置一圈为65536，带方向符号。 */
    uint8_t RxBuffer[32];
    uint8_t PositionFrame[8];
    uint8_t PositionFrameIndex;
    volatile int32_t RealTimePositionRaw;
    volatile uint32_t PositionUpdateTick;
    volatile uint8_t PositionValid;
    volatile uint8_t PositionRequestPending;
    uint32_t PositionRequestTick;
    uint32_t PositionQueryTick;
}Emm_t;

void Emm_Init(Emm_t *Emm, Serial_t *Serial);
void Emm_Set_Addr(Emm_t *Emm, uint8_t Addr);
void Emm_SetSpeed(Emm_t *Emm, uint16_t Vel);
void Emm_SetAcc(Emm_t *Emm, uint8_t Acc);
void Emm_SetMode(Emm_t *Emm, uint8_t raF, bool snF);
void Emm_Pos_Run(Emm_t *Emm, int32_t Pul);
void Emm_Pos_Control_Quick_Init(Emm_t *Emm);
void Emm_Pos_Run_Quick(Emm_t *Emm, int32_t Pul);
uint8_t Emm_Request_RealTime_Position(Emm_t *Emm);
void Emm_Position_Feedback_Loop(Emm_t *Emm,
                                uint32_t Now,
                                uint32_t Period_ms);
uint8_t Emm_Get_RealTime_Position(Emm_t *Emm,
                                  int32_t *RawPosition,
                                  uint32_t *Age_ms);
uint8_t Emm_Get_RealTime_Position_Pulse(Emm_t *Emm,
                                        float *PositionPulse,
                                        uint32_t *Age_ms);

#endif
