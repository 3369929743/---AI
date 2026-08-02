#include "JY61P.h"
#include <stdint.h>
#include "Serial.h"

#define JY61P_RX_BUFFER_SIZE    64
#define JY61P_ACCEL_TYPE         0x51
#define JY61P_PACKET_SIZE        11
#define JY61P_CALIBRATION_SAMPLES 50U
#define JY61P_CALIBRATION_MOTION_THRESHOLD 100

typedef enum {
    JY61P_SYNC = 0,
    JY61P_TYPE,
    JY61P_DATA,
    JY61P_CHECK,
} JY61P_ParseState_e;

static void JY61P_Rx_Event(Serial_t *Serial, uint16_t Size);

typedef struct {
    Serial_t *Serial;
    uint8_t   BufferA[JY61P_RX_BUFFER_SIZE];
    uint8_t   BufferB[JY61P_RX_BUFFER_SIZE];
    uint8_t  *ProcessPtr;
    uint8_t  *ReadPtr;
    uint16_t  Size;
    uint8_t   BufferFlag;

    JY61P_Accel_t Accel;
    uint8_t       AccelFresh;

    int32_t       AccelSumX;
    int32_t       AccelSumY;
    int32_t       AccelSumZ;
    int32_t       AccelZeroX;
    int32_t       AccelZeroY;
    int32_t       AccelZeroZ;
    int16_t       CalibrationPreviousX;
    int16_t       CalibrationPreviousY;
    int16_t       CalibrationPreviousZ;
    uint16_t      CalibrationCount;
    uint8_t       HasCalibrationPrevious;
    uint8_t       IsCalibrated;
} JY61P_Buffer_t;

static JY61P_Buffer_t JY61P_Buffer;

static int32_t JY61P_Abs32(int32_t Value)
{
    return (Value < 0) ? -Value : Value;
}

static int32_t JY61P_Divide_Rounded(int32_t Sum, int32_t Divisor)
{
    if(Sum >= 0){
        return (Sum + Divisor / 2) / Divisor;
    }
    return (Sum - Divisor / 2) / Divisor;
}

static void JY61P_Calibration_Reset_Accumulation(void)
{
    JY61P_Buffer.AccelSumX = 0;
    JY61P_Buffer.AccelSumY = 0;
    JY61P_Buffer.AccelSumZ = 0;
    JY61P_Buffer.CalibrationCount = 0;
}

static void JY61P_Calibration_Update(void)
{
    int32_t DeltaX;
    int32_t DeltaY;
    int32_t DeltaZ;

    if(JY61P_Buffer.IsCalibrated) return;

    if(JY61P_Buffer.HasCalibrationPrevious){
        DeltaX = (int32_t)JY61P_Buffer.Accel.Ax
               - (int32_t)JY61P_Buffer.CalibrationPreviousX;
        DeltaY = (int32_t)JY61P_Buffer.Accel.Ay
               - (int32_t)JY61P_Buffer.CalibrationPreviousY;
        DeltaZ = (int32_t)JY61P_Buffer.Accel.Az
               - (int32_t)JY61P_Buffer.CalibrationPreviousZ;

        /* Restart the average if the platform is moved during calibration. */
        if(JY61P_Abs32(DeltaX) > JY61P_CALIBRATION_MOTION_THRESHOLD
           || JY61P_Abs32(DeltaY) > JY61P_CALIBRATION_MOTION_THRESHOLD
           || JY61P_Abs32(DeltaZ) > JY61P_CALIBRATION_MOTION_THRESHOLD){
            JY61P_Calibration_Reset_Accumulation();
        }
    }

    JY61P_Buffer.CalibrationPreviousX = JY61P_Buffer.Accel.Ax;
    JY61P_Buffer.CalibrationPreviousY = JY61P_Buffer.Accel.Ay;
    JY61P_Buffer.CalibrationPreviousZ = JY61P_Buffer.Accel.Az;
    JY61P_Buffer.HasCalibrationPrevious = 1;

    JY61P_Buffer.AccelSumX += JY61P_Buffer.Accel.Ax;
    JY61P_Buffer.AccelSumY += JY61P_Buffer.Accel.Ay;
    JY61P_Buffer.AccelSumZ += JY61P_Buffer.Accel.Az;
    JY61P_Buffer.CalibrationCount++;

    if(JY61P_Buffer.CalibrationCount >= JY61P_CALIBRATION_SAMPLES){
        JY61P_Buffer.AccelZeroX = JY61P_Divide_Rounded(
            JY61P_Buffer.AccelSumX, JY61P_Buffer.CalibrationCount);
        JY61P_Buffer.AccelZeroY = JY61P_Divide_Rounded(
            JY61P_Buffer.AccelSumY, JY61P_Buffer.CalibrationCount);
        JY61P_Buffer.AccelZeroZ = JY61P_Divide_Rounded(
            JY61P_Buffer.AccelSumZ, JY61P_Buffer.CalibrationCount);
        JY61P_Buffer.IsCalibrated = 1;
    }
}

typedef struct {
    JY61P_ParseState_e State;
    uint8_t  Data[8];
    uint8_t  DataIdx;
    uint8_t  Type;
    uint8_t  Sum;
} JY61P_Parser_t;

static uint8_t JY61P_Parse_Snapshot(const uint8_t *Buf, uint16_t Size)
{
    JY61P_Parser_t Parser = {JY61P_SYNC, {0}, 0, 0, 0};
    uint16_t       i;

    for(i = 0; i < Size; i++){
        switch(Parser.State){
            case JY61P_SYNC:
                if(Buf[i] == 0x55){
                    Parser.State = JY61P_TYPE;
                    Parser.Sum   = 0x55;
                }
                break;

            case JY61P_TYPE:
                Parser.Type  = Buf[i];
                Parser.Sum  += Buf[i];
                if(Parser.Type == JY61P_ACCEL_TYPE){
                    Parser.State   = JY61P_DATA;
                    Parser.DataIdx = 0;
                }
                else{
                    Parser.State = JY61P_SYNC;
                }
                break;

            case JY61P_DATA:
                Parser.Data[Parser.DataIdx] = Buf[i];
                Parser.Sum                 += Buf[i];
                Parser.DataIdx++;
                if(Parser.DataIdx >= 8){
                    Parser.State = JY61P_CHECK;
                }
                break;

            case JY61P_CHECK:
                if(Buf[i] == (Parser.Sum & 0xFF)){
                    JY61P_Buffer.Accel.Ax   = (int16_t)((uint16_t)Parser.Data[1] << 8 | Parser.Data[0]);
                    JY61P_Buffer.Accel.Ay   = (int16_t)((uint16_t)Parser.Data[3] << 8 | Parser.Data[2]);
                    JY61P_Buffer.Accel.Az   = (int16_t)((uint16_t)Parser.Data[5] << 8 | Parser.Data[4]);
                    JY61P_Buffer.Accel.Temp = (int16_t)((uint16_t)Parser.Data[7] << 8 | Parser.Data[6]);
                    JY61P_Buffer.AccelFresh = 1;
                }
                Parser.State = JY61P_SYNC;
                break;
        }
    }
    return JY61P_Buffer.AccelFresh;
}

void JY61P_Init(Serial_t *Serial)
{
    JY61P_Buffer.Serial     = Serial;
    JY61P_Buffer.ProcessPtr = JY61P_Buffer.BufferA;
    JY61P_Buffer.ReadPtr    = JY61P_Buffer.BufferB;
    JY61P_Buffer.BufferFlag = 0;
    JY61P_Buffer.AccelFresh = 0;
    JY61P_Buffer.AccelZeroX = 0;
    JY61P_Buffer.AccelZeroY = 0;
    JY61P_Buffer.AccelZeroZ = 0;
    JY61P_Buffer.HasCalibrationPrevious = 0;
    JY61P_Buffer.IsCalibrated = 0;
    JY61P_Calibration_Reset_Accumulation();

    Serial_SetRxBuffer(Serial, JY61P_Buffer.ProcessPtr, JY61P_RX_BUFFER_SIZE);
    Serial_ReceiveToIdle_IT(Serial);
    Serial->RxCallback = JY61P_Rx_Event;
}

uint8_t JY61P_Accel_Update(void)
{
    uint8_t  Snapshot[JY61P_RX_BUFFER_SIZE];
    uint16_t Size;
    uint32_t Primask;
    uint16_t i;

    Primask = __get_PRIMASK();
    __disable_irq();
    if(!JY61P_Buffer.BufferFlag){
        if(!Primask) __enable_irq();
        return 0;
    }

    Size = JY61P_Buffer.Size;
    if(Size > JY61P_RX_BUFFER_SIZE) Size = JY61P_RX_BUFFER_SIZE;
    for(i = 0; i < Size; i++){
        Snapshot[i] = JY61P_Buffer.ReadPtr[i];
    }
    JY61P_Buffer.BufferFlag = 0;
    if(!Primask) __enable_irq();

    JY61P_Buffer.AccelFresh = 0;
    if(!JY61P_Parse_Snapshot(Snapshot, Size)){
        return 0;
    }

    JY61P_Calibration_Update();
    return 1;
}

void JY61P_Get_Accel(JY61P_Accel_t *Accel)
{
    if(Accel == NULL) return;

    uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    Accel->Ax   = JY61P_Buffer.Accel.Ax;
    Accel->Ay   = JY61P_Buffer.Accel.Ay;
    Accel->Az   = JY61P_Buffer.Accel.Az;
    Accel->Temp = JY61P_Buffer.Accel.Temp;
    if(!Primask) __enable_irq();
}

uint8_t JY61P_Is_Calibrated(void)
{
    return JY61P_Buffer.IsCalibrated;
}

uint16_t JY61P_Get_Calibration_Count(void)
{
    return JY61P_Buffer.CalibrationCount;
}

uint16_t JY61P_Get_Calibration_Target(void)
{
    return JY61P_CALIBRATION_SAMPLES;
}

void JY61P_Get_Calibrated_Accel(JY61P_CalibratedAccel_t *Accel)
{
    if(Accel == NULL) return;

    if(!JY61P_Buffer.IsCalibrated){
        Accel->Ax = 0;
        Accel->Ay = 0;
        Accel->Az = 0;
        return;
    }

    Accel->Ax = (int32_t)JY61P_Buffer.Accel.Ax - JY61P_Buffer.AccelZeroX;
    Accel->Ay = (int32_t)JY61P_Buffer.Accel.Ay - JY61P_Buffer.AccelZeroY;
    Accel->Az = (int32_t)JY61P_Buffer.Accel.Az - JY61P_Buffer.AccelZeroZ;
}

static void JY61P_Rx_Event(Serial_t *Serial, uint16_t Size)
{
    if(Serial != JY61P_Buffer.Serial){
        return;
    }
    JY61P_Buffer.Size    = Size;
    JY61P_Buffer.ReadPtr = JY61P_Buffer.ProcessPtr;
    if(JY61P_Buffer.ProcessPtr == JY61P_Buffer.BufferA){
        JY61P_Buffer.ProcessPtr = JY61P_Buffer.BufferB;
    }
    else{
        JY61P_Buffer.ProcessPtr = JY61P_Buffer.BufferA;
    }
    JY61P_Buffer.BufferFlag = 1;
    Serial_SetRxBuffer(Serial, JY61P_Buffer.ProcessPtr, JY61P_RX_BUFFER_SIZE);
    Serial_ReceiveToIdle_IT(Serial);
}
