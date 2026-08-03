#include "Task_Blue.h"
#include "Serial_Stream.h"
#include "Task_Ball_Contral.h"
#include "main.h"

#define BLUE_DEBUG_PERIOD_MS 50U

typedef enum{
  BLUE_RESPONSE_NONE = 0,
  BLUE_RESPONSE_HEADER,
  BLUE_RESPONSE_DEBUG_OFF,
  BLUE_RESPONSE_DEBUG_ON,
  BLUE_RESPONSE_PONG,
}Blue_Response_e;

static Serial_t Serial_Blue;
static FIFO_t FIFO_Blue;
static Serial_Stream_t Serial_Stream_Blue;
static uint8_t Blue_FIFO_Buff[128];    
static uint8_t Blue_RxTemp_Buff[64];
static uint8_t Blue_User_Buff[64];
static uint32_t Blue_Last_Debug_Tick;
static uint8_t Blue_Debug_Enable;
static uint8_t Blue_Wait_Debug_Command;
static Blue_Response_e Blue_Pending_Response;
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
  Blue_Debug_Enable = 1U;
  Blue_Wait_Debug_Command = 0U;
  Blue_Pending_Response = BLUE_RESPONSE_HEADER;
}

static void Task_Blue_Process_Rx(void){
  uint16_t Size;
  uint16_t Index;

  Size = Serial_Stream_ReadArray(&Serial_Stream_Blue,
                                 Blue_User_Buff,
                                 sizeof(Blue_User_Buff));
  for(Index = 0; Index < Size; Index++){
    uint8_t Byte = Blue_User_Buff[Index];

    if(Blue_Wait_Debug_Command){
      Blue_Wait_Debug_Command = 0U;
      if(Byte == '0'){
        Blue_Debug_Enable = 0U;
        Blue_Pending_Response = BLUE_RESPONSE_DEBUG_OFF;
      }
      else if(Byte == '1'){
        Blue_Debug_Enable = 1U;
        Blue_Last_Debug_Tick = 0U;
        Blue_Pending_Response = BLUE_RESPONSE_DEBUG_ON;
      }
      continue;
    }

    if(Byte == 'D' || Byte == 'd'){
      Blue_Wait_Debug_Command = 1U;
    }
    else if(Byte == 'P' || Byte == 'p'){
      Blue_Pending_Response = BLUE_RESPONSE_PONG;
    }
    else if(Byte == '?'){
      Blue_Pending_Response = BLUE_RESPONSE_HEADER;
    }
  }
}

static uint8_t Task_Blue_Send_Pending_Response(void){
  Blue_Response_e Response = Blue_Pending_Response;

  if(Response == BLUE_RESPONSE_NONE) return 0U;
  Blue_Pending_Response = BLUE_RESPONSE_NONE;

  switch(Response){
    case BLUE_RESPONSE_HEADER:
      Serial_Printf(&Serial_Blue,
                    "H,t,z,e,af,ar,ff,fb,co,ii,pp,tg,mp,ma,bw,fl\r\n");
      break;
    case BLUE_RESPONSE_DEBUG_OFF:
      Serial_Printf(&Serial_Blue, "OK,D0\r\n");
      break;
    case BLUE_RESPONSE_DEBUG_ON:
      Serial_Printf(&Serial_Blue, "OK,D1\r\n");
      break;
    case BLUE_RESPONSE_PONG:
      Serial_Printf(&Serial_Blue, "PONG\r\n");
      break;
    case BLUE_RESPONSE_NONE:
    default:
      return 0U;
  }
  return 1U;
}

void Task_Blue_Loop(void){
  uint32_t Now;
  Task_Ball_Debug_Data_t Debug_Data;

  Task_Blue_Process_Rx();

  /* 不在控制主循环中等待慢速蓝牙发送完成。 */
  if(Serial_Blue.isBusy) return;
  if(Task_Blue_Send_Pending_Response()) return;
  if(!Blue_Debug_Enable) return;

  Now = HAL_GetTick();
  if((uint32_t)(Now - Blue_Last_Debug_Tick) < BLUE_DEBUG_PERIOD_MS) return;
  Blue_Last_Debug_Tick = Now;

  Task_Ball_Contral_Get_Debug_Data(&Debug_Data);
  Serial_Printf(&Serial_Blue,
                "D,%lu,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%ld,%lu,%u,%02X\r\n",
                (unsigned long)(Now % 100000UL),
                (int)Debug_Data.Vision_Zero_x,
                (int)Debug_Data.Vision_Error_x,
                (long)Debug_Data.Accel_Ay_Filtered,
                (long)Debug_Data.Accel_Ay_Highpass,
                (long)Debug_Data.Feedforward_x10,
                (long)Debug_Data.Feedback_x10,
                (long)Debug_Data.Combined_x10,
                (long)Debug_Data.Integral_x10,
                (long)Debug_Data.Pipe_Target_Pulse,
                (int)Debug_Data.Target_x,
                (long)Debug_Data.Motor_Actual_Pulse,
                (unsigned long)Debug_Data.Motor_Position_Age_ms,
                (unsigned int)Debug_Data.Feedforward_Blend_x100,
                (unsigned int)Debug_Data.Flags);
}
