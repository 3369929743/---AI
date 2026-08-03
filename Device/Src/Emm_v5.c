#include "Emm_v5.h"
#include "Serial.h"
#include <stdint.h>

#define CHECK_SUM 0x6B // 校验和值

#define EMM_POS_CONTROL 0xFD // 位置控制功能码
#define EMM_POS_CONTROL_QUICK_CHOOSE 0xF1 // 位置控制快速选择功能码
#define EMM_POS_CONTROL_QUICK 0xFC // 位置控制快速功能码
#define EMM_POS_RESET 0x0A // 位置重置功能码
#define EMM_POS_RESER_ASSIST 0x6D // 位置重置辅助功能码
#define EMM_SPEED_CONTROL 0xF6
#define EMM_READ_REALTIME_POSITION 0x36

#define DEFAULT_SPEED 50
#define EMM_POSITION_RESPONSE_SIZE 8U
#define EMM_POSITION_REQUEST_TIMEOUT_MS 30U
#define EMM_POSITION_RAW_PER_REV 65536.0f
#define EMM_CONTROL_PULSE_PER_REV 3200.0f

static void Emm_Rx_Event(Serial_t *Serial, uint16_t Size);

static void Emm_Position_Parse_Byte(Emm_t *Emm, uint8_t Byte)
{
    uint8_t Index = Emm->PositionFrameIndex;

    if(Index == 0U){
        if(Byte == Emm->Addr){
            Emm->PositionFrame[0] = Byte;
            Emm->PositionFrameIndex = 1U;
        }
        return;
    }

    if(Index == 1U){
        if(Byte == EMM_READ_REALTIME_POSITION){
            Emm->PositionFrame[1] = Byte;
            Emm->PositionFrameIndex = 2U;
        }
        else if(Byte != Emm->Addr){
            Emm->PositionFrameIndex = 0U;
        }
        return;
    }

    Emm->PositionFrame[Index] = Byte;
    Index++;
    Emm->PositionFrameIndex = Index;
    if(Index < EMM_POSITION_RESPONSE_SIZE) return;

    Emm->PositionFrameIndex = 0U;
    if(Emm->PositionFrame[7] == CHECK_SUM
       && Emm->PositionFrame[2] <= 1U){
        uint32_t Magnitude =
            ((uint32_t)Emm->PositionFrame[3] << 24)
          | ((uint32_t)Emm->PositionFrame[4] << 16)
          | ((uint32_t)Emm->PositionFrame[5] << 8)
          |  (uint32_t)Emm->PositionFrame[6];
        int32_t Position;

        if(Magnitude > 0x7FFFFFFFUL) Magnitude = 0x7FFFFFFFUL;
        Position = (int32_t)Magnitude;
        if(Emm->PositionFrame[2]) Position = -Position;

        Emm->RealTimePositionRaw = Position;
        Emm->PositionUpdateTick = HAL_GetTick();
        Emm->PositionValid = 1U;
        Emm->PositionRequestPending = 0U;
    }
}

static void Emm_Rx_Event(Serial_t *Serial, uint16_t Size)
{
    Emm_t *Emm;
    uint16_t Index;

    if(Serial == 0 || Serial->User_Data == 0) return;
    Emm = (Emm_t *)Serial->User_Data;
    if(Emm->Serial != Serial) return;

    if(Size > sizeof(Emm->RxBuffer)) Size = sizeof(Emm->RxBuffer);
    for(Index = 0U; Index < Size; Index++){
        Emm_Position_Parse_Byte(Emm, Emm->RxBuffer[Index]);
    }

    Serial_SetRxBuffer(Serial, Emm->RxBuffer, sizeof(Emm->RxBuffer));
    Serial_ReceiveToIdle_IT(Serial);
}

/**
  * @brief  EMM-V5电机驱动器初始化函数
  * @param  Emm: EMM位置控制结构体指针，用于存储电机控制参数和状态
  * @param  Serial: 串口通信结构体指针，用于与电机驱动器进行串口通信
  * @retval 无
  * @note   - 初始化后默认参数：地址0x01、方向CW、速度50、加速度0、相对模式、不同步
  *         - 可通过Emm_SetSpeed/Emm_SetAcc/Emm_SetMode等函数修改参数
  */
void Emm_Init(Emm_t *Emm, Serial_t *Serial)
{
	Emm->Serial = Serial;      /* 绑定串口通信接口：用于向电机驱动器发送控制指令 */
	Emm->Addr = 0x01;          /* 设置默认电机驱动器地址：多机通信时用于区分目标设备 */

	Emm->Dir = 0;              /* 默认旋转方向：0表示CW（顺时针），1表示CCW（逆时针） */
	Emm->Vel = DEFAULT_SPEED;  /* 默认运行速度：单位由电机驱动器定义（通常为RPM或脉冲/秒） */
	Emm->Acc = 0;              /* 默认加速度：0表示使用驱动器默认加速度曲线 */
	Emm->raF = 0;              /* 默认位置模式：0表示相对模式（相对上一目标位置），1表示绝对模式（相对坐标零点） */
	Emm->snF = 0;              /* 默认同步模式：0表示不同步，1表示多机同步运动 */
	Emm->PositionFrameIndex = 0U;
	Emm->RealTimePositionRaw = 0;
	Emm->PositionUpdateTick = 0U;
	Emm->PositionValid = 0U;
	Emm->PositionRequestPending = 0U;
	Emm->PositionRequestTick = 0U;
	Emm->PositionQueryTick = 0U;
	Serial->User_Data = (uint8_t *)Emm;
	Serial_SetRxBuffer(Serial, Emm->RxBuffer, sizeof(Emm->RxBuffer));
	Serial->RxCallback = Emm_Rx_Event;
	Serial_ReceiveToIdle_IT(Serial);
}

uint8_t Emm_Request_RealTime_Position(Emm_t *Emm)
{
    uint8_t Command[3];
    uint32_t Now;

    if(Emm == 0 || Emm->Serial == 0) return 0U;
    Now = HAL_GetTick();
    if(Emm->PositionRequestPending
       && (uint32_t)(Now - Emm->PositionRequestTick)
          < EMM_POSITION_REQUEST_TIMEOUT_MS){
        return 0U;
    }
    if(Emm->Serial->isBusy) return 0U;

    Command[0] = Emm->Addr;
    Command[1] = EMM_READ_REALTIME_POSITION;
    Command[2] = CHECK_SUM;
    if(Serial_SendArray(Emm->Serial, Command, sizeof(Command)) != 0U){
        return 0U;
    }

    Emm->PositionRequestPending = 1U;
    Emm->PositionRequestTick = Now;
    return 1U;
}

void Emm_Position_Feedback_Loop(Emm_t *Emm,
                                uint32_t Now,
                                uint32_t Period_ms)
{
    if(Emm == 0 || Period_ms == 0U) return;
    if((uint32_t)(Now - Emm->PositionQueryTick) < Period_ms) return;

    if(Emm_Request_RealTime_Position(Emm)){
        Emm->PositionQueryTick = Now;
    }
}

uint8_t Emm_Get_RealTime_Position(Emm_t *Emm,
                                  int32_t *RawPosition,
                                  uint32_t *Age_ms)
{
    uint32_t Primask;
    uint32_t UpdateTick;
    int32_t Position;
    uint8_t Valid;

    if(Emm == 0) return 0U;
    Primask = __get_PRIMASK();
    __disable_irq();
    Position = Emm->RealTimePositionRaw;
    UpdateTick = Emm->PositionUpdateTick;
    Valid = Emm->PositionValid;
    if(!Primask) __enable_irq();

    if(RawPosition != 0) *RawPosition = Position;
    if(Age_ms != 0) *Age_ms = HAL_GetTick() - UpdateTick;
    return Valid;
}

uint8_t Emm_Get_RealTime_Position_Pulse(Emm_t *Emm,
                                        float *PositionPulse,
                                        uint32_t *Age_ms)
{
    int32_t RawPosition;
    uint32_t Age;
    uint8_t Valid = Emm_Get_RealTime_Position(Emm,
                                              &RawPosition,
                                              &Age);

    if(PositionPulse != 0){
        *PositionPulse = (float)RawPosition
                       * EMM_CONTROL_PULSE_PER_REV
                       / EMM_POSITION_RAW_PER_REV;
    }
    if(Age_ms != 0) *Age_ms = Age;
    return Valid;
}

void Emm_Set_Addr(Emm_t * Emm, uint8_t Addr){
  Emm->Addr = Addr;
}

/**
  * @brief  设置电机旋转方向（内部函数）
  * @param  Emm: EMM位置控制结构体指针
  * @param  Dir: 旋转方向（0：CW顺时针，1：CCW逆时针）
  * @retval 无
  * @note   此函数为static inline内部函数，通常由Emm_Pos_Control内部自动调用
  */
static inline void Emm_SetDir(Emm_t *Emm, uint8_t Dir)
{
    Emm->Dir = Dir;  /* 更新电机旋转方向：0表示CW（顺时针），1表示CCW（逆时针） */
}

/**
  * @brief  设置电机运行速度
  * @param  Emm: EMM位置控制结构体指针
  * @param  Vel: 目标速度值（单位由电机驱动器定义，通常为RPM或脉冲/秒）
  * @retval 无
  * @note   速度值需根据电机驱动器的规格设置，过大可能导致失步或过载
  */
void Emm_SetSpeed(Emm_t *Emm, uint16_t Vel)
{
    Emm->Vel = Vel;  /* 更新电机运行速度：写入目标速度值，用于后续位置控制指令 */
}

/**
  * @brief  设置电机加速度
  * @param  Emm: EMM位置控制结构体指针
  * @param  Acc: 加速度值（0表示使用驱动器默认加速度曲线）
  * @retval 无
  * @note   - 加速度值影响电机启动和停止的平滑度
  *         - 值越大加减速越快，但过大可能导致失步或机械冲击
  *         - 设置为0时使用电机驱动器内置的默认加速度曲线
  */
void Emm_SetAcc(Emm_t *Emm, uint8_t Acc)
{
    Emm->Acc = Acc;  /* 更新电机加速度：写入加速度值，用于控制电机加减速过程 */
}

/**
  * @brief  设置电机运行模式（位置模式+同步模式）
  * @param  Emm: EMM位置控制结构体指针
  * @param  raF: 相对/绝对位置模式标志
  *               - 0：相对模式（相对上一目标位置进行位置运动）
  *               - 1：绝对模式（相对坐标零点进行绝对位置运动）
  *               - 2：相对模式（相对当前实时位置进行位置运动）
  * @param  snF: 多机同步运动标志（false：不同步，true：同步）
  * @retval 无
  * @note   - 相对模式适用于增量式位置调整，绝对模式适用于精确定位到固定坐标点
  *         - 同步模式用于多台电机协同运动，确保各电机同时启动和停止
  */
void Emm_SetMode(Emm_t *Emm, uint8_t raF, bool snF)
{
    Emm->raF = raF;      /* 更新位置模式标志：0=相对上一位置，1=绝对坐标零点，2=相对当前实时位置 */
    Emm->snF = snF;      /* 更新同步模式标志：false表示独立运行，true表示多机同步协调运动 */
}

/**
  * @brief  设置电机目标脉冲数（内部函数）
  * @param  Emm: EMM位置控制结构体指针
  * @param  Pul: 目标脉冲数（正数表示正转，负数表示反转）
  * @retval 无
  * @note   - 脉冲数决定电机旋转角度，通常3200脉冲=电机转一圈
  *         - 此函数为static inline内部函数，由Emm_Pos_Run/Emm_Pos_Run_Quick调用
  *         - 脉冲数的符号决定旋转方向，绝对值决定旋转角度
  */
static inline void Emm_SetPul(Emm_t *Emm, int32_t Pul)
{
    Emm->Pul = Pul;  /* 更新目标脉冲数：正负号决定方向，绝对值决定旋转角度 */
}

/**
  * @brief  EMM-V5位置控制指令发送函数（内部函数）
  * @param  Emm: EMM位置控制结构体指针
  * @retval 无
  * @note   - 构建13字节位置控制指令帧并通过串口发送给电机驱动器
  *         - 指令格式：[地址][功能码][方向][速度高8位][速度低8位][加速度]
  *                    [脉冲数24-31位][脉冲数16-23位][脉冲数8-15位][脉冲数0-7位]
  *                    [相对/绝对标志][同步标志][校验和]
  *         - 通常3200脉冲=电机转一圈
  * @warning 此函数为static内部函数，需确保Emm结构体参数已正确配置
  */
static void Emm_Pos_Control(Emm_t *Emm)
{
	uint8_t cmd[16] = {0};              /* 指令缓冲区：最大16字节，实际使用13字节 */
	int32_t abs_Pul = Emm->Pul;         /* 取脉冲数的绝对值：用于指令帧中的脉冲数据 */
	
	/* 根据脉冲数符号确定旋转方向 */
	if(abs_Pul >= 0)
	{
		Emm_SetDir(Emm, 0);             /* 脉冲数>=0：设置为CW（顺时针）方向 */
	}
	else
	{
		Emm_SetDir(Emm, 1);             /* 脉冲数<0：设置为CCW（逆时针）方向 */
		abs_Pul = -abs_Pul;             /* 取脉冲数的绝对值 */
	}

	/* ========== 构建位置控制指令帧 ========== */
	cmd[0] = Emm->Addr;                 /* 字节0：电机驱动器地址（用于多机通信区分设备） */
	cmd[1] = EMM_POS_CONTROL;           /* 字节1：功能码0xFD（位置控制模式） */
	cmd[2] = Emm->Dir;                  /* 字节2：旋转方向（0=CW，1=CCW） */
	cmd[3] = (uint8_t)(Emm->Vel >> 8);  /* 字节3：速度高8位（大端模式） */
	cmd[4] = (uint8_t)(Emm->Vel >> 0);  /* 字节4：速度低8位 */
	cmd[5] = Emm->Acc;                  /* 字节5：加速度值 */
	cmd[6] = (uint8_t)(abs_Pul >> 24);  /* 字节6：脉冲数最高8位（bit24-31） */
	cmd[7] = (uint8_t)(abs_Pul >> 16);  /* 字节7：脉冲数次高8位（bit16-23） */
	cmd[8] = (uint8_t)(abs_Pul >> 8);   /* 字节8：脉冲数次低8位（bit8-15） */
	cmd[9] = (uint8_t)(abs_Pul >> 0);   /* 字节9：脉冲数最低8位（bit0-7） */
	cmd[10] = Emm->raF;                 /* 字节10：相对/绝对位置模式标志 */
	cmd[11] = Emm->snF;                 /* 字节11：多机同步运动标志 */
	cmd[12] = CHECK_SUM;                /* 字节12：校验和（固定值0x6B） */
		
	/* 通过串口发送13字节指令帧到电机驱动器 */
	Serial_SendArray(Emm->Serial, cmd, 13);
}		

/**
  * @brief  EMM-V5位置控制快速模式初始化函数
  * @param  Emm: EMM位置控制结构体指针
  * @retval 无
  * @note   - 发送8字节快速模式初始化指令，配置速度、加速度和运行模式
  *         - 初始化后只需发送脉冲数即可执行位置控制，减少通信数据量
  *         - 指令格式：[地址][功能码0xF1][速度高8位][速度低8位][加速度]
  *                    [相对/绝对标志][同步标志][校验和]
  * @warning 调用此函数后需配合Emm_Pos_Run_Quick使用，先初始化再发送脉冲
  */
void Emm_Pos_Control_Quick_Init(Emm_t *Emm)
{
	uint8_t cmd[8] = {0};               /* 指令缓冲区：8字节快速模式初始化指令 */
	
	/* ========== 构建快速模式初始化指令帧 ========== */
	cmd[0] = Emm->Addr;                 /* 字节0：电机驱动器地址 */
	cmd[1] = EMM_POS_CONTROL_QUICK_CHOOSE;  /* 字节1：功能码0xF1（位置控制快速模式选择） */
	cmd[2] = (uint8_t)(Emm->Vel >> 8);  /* 字节2：速度高8位（大端模式） */
	cmd[3] = (uint8_t)(Emm->Vel >> 0);  /* 字节3：速度低8位 */
	cmd[4] = Emm->Acc;                  /* 字节4：加速度值 */
	cmd[5] = Emm->raF;                  /* 字节5：相对/绝对位置模式标志 */
	cmd[6] = Emm->snF;                  /* 字节6：多机同步运动标志 */
	cmd[7] = CHECK_SUM;                 /* 字节7：校验和（固定值0x6B） */
		
	/* 通过串口发送8字节初始化指令帧到电机驱动器 */
	Serial_SendArray(Emm->Serial, cmd, 8);
}

/**
  * @brief  EMM-V5位置控制快速模式指令发送函数（内部函数）
  * @param  Emm: EMM位置控制结构体指针
  * @retval 无
  * @note   - 发送7字节快速模式位置控制指令，仅包含地址、功能码、脉冲数和校验和
  *         - 需先调用Emm_Pos_Control_Quick_Init初始化速度、加速度和模式参数
  *         - 指令格式：[地址][功能码0xFC][脉冲数24-31位][脉冲数16-23位]
  *                    [脉冲数8-15位][脉冲数0-7位][校验和]
  * @warning 此函数为static内部函数，需确保已通过Quick_Init配置好参数
  */
static void Emm_Pos_Control_Quick(Emm_t *Emm)
{
	uint8_t cmd[7] = {0};               /* 指令缓冲区：7字节快速模式位置控制指令 */

	/* ========== 构建快速模式位置控制指令帧 ========== */
	cmd[0] = Emm->Addr;                 /* 字节0：电机驱动器地址 */
	cmd[1] = EMM_POS_CONTROL_QUICK;     /* 字节1：功能码0xFC（位置控制快速模式） */
	cmd[2] = (uint8_t)(Emm->Pul >> 24); /* 字节2：脉冲数最高8位（bit24-31） */
	cmd[3] = (uint8_t)(Emm->Pul >> 16); /* 字节3：脉冲数次高8位（bit16-23） */
	cmd[4] = (uint8_t)(Emm->Pul >> 8);  /* 字节4：脉冲数次低8位（bit8-15） */
	cmd[5] = (uint8_t)(Emm->Pul >> 0);  /* 字节5：脉冲数最低8位（bit0-7） */
	cmd[6] = CHECK_SUM;                 /* 字节6：校验和（固定值0x6B） */
		
	/* 通过串口发送7字节指令帧到电机驱动器 */
	Serial_SendArray(Emm->Serial, cmd, 7);
}

/**
  * @brief  电机位置控制运行函数（标准模式）
  * @param  Emm: EMM位置控制结构体指针
  * @param  Pul: 目标脉冲数（正数表示正转，负数表示反转）
  * @retval 无
  * @note   - 一步到位的位置控制接口：内部自动设置脉冲数并发送完整控制指令
  *         - 每次调用都会发送13字节完整指令帧（包含地址、方向、速度、加速度、脉冲数等）
  *         - 适用于单次位置控制或参数频繁变化的场景
  * @warning 调用前需确保已通过Emm_Init/Emm_SetSpeed/Emm_SetAcc等函数配置好参数
  */
void Emm_Pos_Run(Emm_t *Emm, int32_t Pul)
{
	Emm_SetPul(Emm, Pul);           /* 设置目标脉冲数：正负号决定方向，绝对值决定旋转角度 */
	Emm_Pos_Control(Emm);           /* 发送完整位置控制指令帧（13字节）到电机驱动器 */
}

/**
  * @brief  电机位置控制运行函数（快速模式）
  * @param  Emm: EMM位置控制结构体指针
  * @param  Pul: 目标脉冲数（正数表示正转，负数表示反转）
  * @retval 无
  * @note   - 一步到位的快速模式位置控制接口：内部自动设置脉冲数并发送精简控制指令
  *         - 每次调用仅发送7字节指令帧（仅包含地址、功能码、脉冲数和校验和）
  *         - 适用于需要频繁发送位置指令的场景，通信效率更高
  * @warning - 调用前需先调用Emm_Pos_Control_Quick_Init初始化速度、加速度和模式参数
  *          - 脉冲数的符号决定旋转方向，绝对值决定旋转角度
  */
void Emm_Pos_Run_Quick(Emm_t *Emm, int32_t Pul)
{
	Emm_SetPul(Emm, Pul);           /* 设置目标脉冲数：正负号决定方向，绝对值决定旋转角度 */
	Emm_Pos_Control_Quick(Emm);     /* 发送快速模式位置控制指令帧（7字节）到电机驱动器 */
}

/**
  * @brief  重置电机位置
  * @param  Emm: EMM位置控制结构体指针
  * @retval 无
  * @note   发送位置重置指令到电机驱动器，将当前位置清零或恢复到初始状态
  */
void Emm_Reset_Pos(Emm_t *Emm){
  uint8_t cmd[16] = {0};

  cmd[0] = Emm->Addr;          /* 电机驱动器地址：指定目标设备 */
  cmd[1] = EMM_POS_RESET;      /* 位置重置功能码：0x0A */
  cmd[2] = EMM_POS_RESER_ASSIST; /* 位置重置辅助功能码：0x6D */
  cmd[3] = CHECK_SUM;          /* 校验和：用于数据完整性验证 */

  Serial_SendArray(Emm->Serial, cmd, 4);
}

/**
  * @brief  电机速度控制函数
  * @param  Emm: EMM位置控制结构体指针
  * @param  Vel: 目标速度值（正数表示CW方向，负数表示CCW方向）
  * @retval 无
  * @note   - 速度值的正负决定电机旋转方向，绝对值决定速度大小
  *         - 发送8字节速度控制指令帧到电机驱动器
  */
void Emm_Speed_Comtrol(Emm_t *Emm, int16_t Vel){
  uint8_t cmd[16] = {0};

  if(Vel >= 0)  Emm->Dir = 0;  /* 速度为正数时设置为CW（顺时针）方向 */
  else          Emm->Dir = 1;  /* 速度为负数时设置为CCW（逆时针）方向 */

  cmd[0] = Emm->Addr;          /* 电机驱动器地址：指定目标设备 */
  cmd[1] = EMM_SPEED_CONTROL;  /* 速度控制功能码：0xF6 */
  cmd[2] = Emm->Dir;           /* 旋转方向：0=CW，1=CCW */
  cmd[3] = (uint8_t)(Vel >> 8); /* 速度值高8位：16位速度的高位字节 */
  cmd[4] = (uint8_t)(Vel >> 0); /* 速度值低8位：16位速度的低位字节 */
  cmd[5] = Emm->Acc;           /* 加速度值：控制电机加减速过程 */
  cmd[6] = Emm->snF;           /* 同步模式标志：false=独立运行，true=多机同步 */
  cmd[7] = CHECK_SUM;          /* 校验和：用于数据完整性验证 */
  
  Serial_SendArray(Emm->Serial, cmd, 8);
}
