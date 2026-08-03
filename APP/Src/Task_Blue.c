#include "Task_Blue.h"
#include "Serial_Stream.h"
#include "Task_Ball_Contral.h"

#define BLUE_DEBUG_PERIOD_MS 50U

static Serial_t Serial_Blue;
static FIFO_t FIFO_Blue;
static Serial_Stream_t Serial_Stream_Blue;
static uint8_t Blue_FIFO_Buff[128];    
static uint8_t Blue_RxTemp_Buff[64];
static uint8_t Blue_User_Buff[32];
static uint32_t Blue_Last_Debug_Tick;
static Serial_Stream_Confg_t Serial_Stream_Blue_Confg ={
  .Serial = &Serial_Blue,
  .FIFO = &FIFO_Blue,
  .FIFO_Buff = Blue_FIFO_Buff,
  .Rx_Temp_Buff = Blue_RxTemp_Buff,
  .Serial_Num = Serial_4,
  .FIFO_Size = sizeof(Blue_FIFO_Buff),
  .Rx_Temp_Buff_Size = sizeof(Blue_RxTemp_Buff),
};

void Task_Blue_Init(void){
  Serial_Stream_Init(&Serial_Stream_Blue, &Serial_Stream_Blue_Confg);
  Blue_Last_Debug_Tick = HAL_GetTick();
  Serial_SendString(&Serial_Blue,
                    "H,t,ay,day,out10,state,action,count,error,pipe\r\n");
}

void Task_Blue_Loop(void){
  Task_Ball_Impact_Debug_t Debug;
  uint32_t Now = HAL_GetTick();

  /* 暂时不解析蓝牙下行命令，但持续清空接收FIFO，UART4收发都保持可用。 */
  (void)Serial_Stream_ReadArray(&Serial_Stream_Blue,
                                Blue_User_Buff,
                                sizeof(Blue_User_Buff));

  if((uint32_t)(Now - Blue_Last_Debug_Tick) < BLUE_DEBUG_PERIOD_MS) return;
  Blue_Last_Debug_Tick = Now;

  Task_Ball_Contral_Get_Impact_Debug(&Debug);
  Serial_Printf(&Serial_Blue,
                "D,%lu,%ld,%ld,%ld,%u,%u,%lu,%ld,%ld\r\n",
                (unsigned long)Now,
                (long)Debug.AccelerationY,
                (long)Debug.AccelerationDeltaY,
                (long)Debug.FixedOutputX10,
                (unsigned int)Debug.ImpactState,
                (unsigned int)Debug.ImpactAction,
                (unsigned long)Debug.TriggerCount,
                (long)Debug.PositionError,
                (long)Debug.PipeTargetPulse);
}
