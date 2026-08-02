#include "Task_Serial.h"
#include "Serial.h"
#include "Serial_Stream.h"

static uint8_t Pop_Enable_FLAG;

Serial_t Serial_Pop_Enable;
FIFO_t FIFO_Pop_Enable;
Serial_Stream_t Serial_Stream_Pop_Enable;

uint8__t FIFO_Pop_Enable_Buff[128];
uint8_t RxTemp_Pop_Enable_Buff[64];
uint8_t Pop_Enable_User_Buff[64];

Serial_Stream_Confg_t Serial_Stream_Pop_Enable_Confg = {
    .Serial = &Serial_Pop_Enable,
    .FIFO = &FIFO_Pop_Enable,
    .FIFO_Buff = FIFO_Pop_Enable_Buff,
    .Rx_Temp_Buff = RxTemp_Pop_Enable_Buff,
    .Serial_Num = Serial_4,
    .FIFO_Size = sizeof(FIFO_Pop_Enable_Buff),
    .Rx_Temp_Buff_Size = sizeof(RxTemp_Pop_Enable_Buff),
};

void Task_Serial_Init(void){
    Serial_Stream_Init(&Serial_Stream_Pop_Enable, &Serial_Stream_Pop_Enable_Confg);
}

uint8_t Task_Serial_Get_Pop_Enable(void){
    uint8_t Pop_Enable_FLAG_Temp = Pop_Enable_FLAG;
    Pop_Enable_FLAG = 0;
    return Pop_Enable_FLAG_Temp;
}

void Task_Serial_Loop(void){
    if(Serial_Stream_ReadArray(&Serial_Stream_Pop_Enable, Pop_Enable_User_Buff, sizeof(Pop_Enable_User_Buff))){
        if(Pop_Enable_User_Buff[0] == 0xAA){
            if(Pop_Enable_User_Buff[1] == 0x55){
                if(Pop_Enable_User_Buff[2] == '1'){
                    Pop_Enable_FLAG = 1;
                }
                else{
                    return;
                }
            }
            else{
                return;
            }
        }
    }
}
