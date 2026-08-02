// 通信协议：0xAA 0x55 0x00 小车停止
// 通信协议：0xAA 0x55 0x01 小车启动
// 通信协议：0xAA 0x55 0x03 回复启动前馈完成
// 通信协议：0xAA 0x55 0x04 回复停止前馈完成

#include "Task_Serial.h"
#include "Serial.h"
#include "Serial_Stream.h"

typedef enum{
    TASK_SERIAL_WAIT_HEAD_AA = 0,
    TASK_SERIAL_WAIT_HEAD_55,
    TASK_SERIAL_WAIT_MOTOR_STATE,
}Task_Serial_Parse_State_e;

static uint8_t Pop_Enable_FLAG;
static uint8_t Car_Motor_State;
static uint8_t Car_Motor_State_Valid;
static uint8_t Car_Motor_State_Event;
static Task_Serial_Parse_State_e Task_Serial_Parse_State;

Serial_t Serial_Pop_Enable;
FIFO_t FIFO_Pop_Enable;
Serial_Stream_t Serial_Stream_Pop_Enable;

uint8_t FIFO_Pop_Enable_Buff[128];
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
    Pop_Enable_FLAG = 0;
    Car_Motor_State = 0;
    Car_Motor_State_Valid = 0;
    Car_Motor_State_Event = 0;
    Task_Serial_Parse_State = TASK_SERIAL_WAIT_HEAD_AA;
}

uint8_t Task_Serial_Get_Pop_Enable(void){
    uint8_t Pop_Enable_FLAG_Temp = Pop_Enable_FLAG;
    Pop_Enable_FLAG = 0;
    return Pop_Enable_FLAG_Temp;
}

uint8_t Task_Serial_Get_Car_Motor_State(uint8_t *IsRunning){
    if(IsRunning == 0 || !Car_Motor_State_Event) return 0;

    *IsRunning = Car_Motor_State;
    Car_Motor_State_Event = 0;
    return 1;
}

uint8_t Task_Serial_Send_Feedforward_Complete(void){
    uint8_t Reply[3] = {0xAAU, 0x55U, 0x03U};

    /* Serial_SendArray 会先复制到串口内部发送缓冲区。 */
    return Serial_SendArray(&Serial_Pop_Enable, Reply, sizeof(Reply));
}

static void Task_Serial_Update_Car_Motor_State(uint8_t State){
    if(State > 1U) return;

    /* 保留旧接口：收到数字 1 时产生一次启动标志。 */
    if(State == 1U){
        Pop_Enable_FLAG = 1;
    }

    /* 重复发送相同状态不会反复触发前馈。 */
    if(!Car_Motor_State_Valid || State != Car_Motor_State){
        Car_Motor_State = State;
        Car_Motor_State_Valid = 1;
        Car_Motor_State_Event = 1;
    }
}

void Task_Serial_Loop(void){
    uint16_t Size;
    uint16_t Index;

    Size = Serial_Stream_ReadArray(&Serial_Stream_Pop_Enable,
                                   Pop_Enable_User_Buff,
                                   sizeof(Pop_Enable_User_Buff));
    for(Index = 0; Index < Size; Index++){
        uint8_t Byte = Pop_Enable_User_Buff[Index];

        switch(Task_Serial_Parse_State){
            case TASK_SERIAL_WAIT_HEAD_AA:
                if(Byte == 0xAAU){
                    Task_Serial_Parse_State = TASK_SERIAL_WAIT_HEAD_55;
                }
                break;

            case TASK_SERIAL_WAIT_HEAD_55:
                if(Byte == 0x55U){
                    Task_Serial_Parse_State = TASK_SERIAL_WAIT_MOTOR_STATE;
                }
                else if(Byte != 0xAAU){
                    Task_Serial_Parse_State = TASK_SERIAL_WAIT_HEAD_AA;
                }
                break;

            case TASK_SERIAL_WAIT_MOTOR_STATE:
                Task_Serial_Update_Car_Motor_State(Byte);
                Task_Serial_Parse_State = TASK_SERIAL_WAIT_HEAD_AA;
                break;

            default:
                Task_Serial_Parse_State = TASK_SERIAL_WAIT_HEAD_AA;
                break;
        }
    }
}
