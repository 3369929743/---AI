#include "Ball_Impact_Config.h"
#include "main.h"

/*
 * STM32F407VE 的最后一个 128 KB Flash 扇区：0x08060000~0x0807FFFF。
 * 链接脚本已将应用程序区域限制到前 384 KB，保证程序不会覆盖这里。
 */
#define BALL_IMPACT_CONFIG_FLASH_ADDRESS    0x08060000U
#define BALL_IMPACT_CONFIG_FLASH_SECTOR     FLASH_SECTOR_7
#define BALL_IMPACT_CONFIG_MAGIC            0x42494346U  /* "BICF" */
#define BALL_IMPACT_CONFIG_VERSION          1U
#define BALL_IMPACT_CONFIG_CHECK_SEED       0xA55A3CC3U
#define BALL_IMPACT_DEFAULT_START_LOWER     25U
#define BALL_IMPACT_DEFAULT_STOP_RAISE      25U

typedef struct{
    uint32_t Magic;
    uint32_t Version;
    uint32_t StartLowerPulse;
    uint32_t StopRaisePulse;
    uint32_t Check;
}BallImpactConfig_FlashRecord_t;

static uint16_t BallImpactConfig_StartLowerPulse =
    BALL_IMPACT_DEFAULT_START_LOWER;
static uint16_t BallImpactConfig_StopRaisePulse =
    BALL_IMPACT_DEFAULT_STOP_RAISE;
static uint8_t BallImpactConfig_Dirty = 1U;
static uint8_t BallImpactConfig_SaveError = 0U;

static uint32_t BallImpactConfig_Calculate_Check(
    const BallImpactConfig_FlashRecord_t *Record)
{
    return Record->Magic
         ^ Record->Version
         ^ Record->StartLowerPulse
         ^ Record->StopRaisePulse
         ^ BALL_IMPACT_CONFIG_CHECK_SEED;
}

static uint8_t BallImpactConfig_Pulse_Is_Valid(uint32_t Pulse)
{
    return Pulse >= BALL_IMPACT_CONFIG_MIN_PULSE
        && Pulse <= BALL_IMPACT_CONFIG_MAX_PULSE
        && (Pulse % BALL_IMPACT_CONFIG_STEP_PULSE) == 0U;
}

static uint8_t BallImpactConfig_Record_Is_Valid(
    const BallImpactConfig_FlashRecord_t *Record)
{
    if(Record->Magic != BALL_IMPACT_CONFIG_MAGIC
       || Record->Version != BALL_IMPACT_CONFIG_VERSION){
        return 0U;
    }
    if(!BallImpactConfig_Pulse_Is_Valid(Record->StartLowerPulse)
       || !BallImpactConfig_Pulse_Is_Valid(Record->StopRaisePulse)){
        return 0U;
    }
    return Record->Check == BallImpactConfig_Calculate_Check(Record);
}

static uint16_t BallImpactConfig_Adjust_Value(uint16_t Value,
                                               int16_t DeltaPulse)
{
    int32_t Adjusted = (int32_t)Value + (int32_t)DeltaPulse;

    if(Adjusted < (int32_t)BALL_IMPACT_CONFIG_MIN_PULSE){
        Adjusted = (int32_t)BALL_IMPACT_CONFIG_MIN_PULSE;
    }
    else if(Adjusted > (int32_t)BALL_IMPACT_CONFIG_MAX_PULSE){
        Adjusted = (int32_t)BALL_IMPACT_CONFIG_MAX_PULSE;
    }
    return (uint16_t)Adjusted;
}

void BallImpactConfig_Init(void)
{
    const BallImpactConfig_FlashRecord_t *Record =
        (const BallImpactConfig_FlashRecord_t *)
        BALL_IMPACT_CONFIG_FLASH_ADDRESS;

    BallImpactConfig_SaveError = 0U;
    if(BallImpactConfig_Record_Is_Valid(Record)){
        BallImpactConfig_StartLowerPulse =
            (uint16_t)Record->StartLowerPulse;
        BallImpactConfig_StopRaisePulse =
            (uint16_t)Record->StopRaisePulse;
        BallImpactConfig_Dirty = 0U;
    }
    else{
        BallImpactConfig_StartLowerPulse =
            BALL_IMPACT_DEFAULT_START_LOWER;
        BallImpactConfig_StopRaisePulse =
            BALL_IMPACT_DEFAULT_STOP_RAISE;
        BallImpactConfig_Dirty = 1U;
    }
}

void BallImpactConfig_Adjust_Start_Lower(int16_t DeltaPulse)
{
    uint16_t Adjusted = BallImpactConfig_Adjust_Value(
        BallImpactConfig_StartLowerPulse, DeltaPulse);

    if(Adjusted != BallImpactConfig_StartLowerPulse){
        BallImpactConfig_StartLowerPulse = Adjusted;
        BallImpactConfig_Dirty = 1U;
        BallImpactConfig_SaveError = 0U;
    }
}

void BallImpactConfig_Adjust_Stop_Raise(int16_t DeltaPulse)
{
    uint16_t Adjusted = BallImpactConfig_Adjust_Value(
        BallImpactConfig_StopRaisePulse, DeltaPulse);

    if(Adjusted != BallImpactConfig_StopRaisePulse){
        BallImpactConfig_StopRaisePulse = Adjusted;
        BallImpactConfig_Dirty = 1U;
        BallImpactConfig_SaveError = 0U;
    }
}

uint8_t BallImpactConfig_Save(void)
{
    BallImpactConfig_FlashRecord_t Record;
    const uint32_t *Words;
    const BallImpactConfig_FlashRecord_t *SavedRecord;
    FLASH_EraseInitTypeDef Erase;
    uint32_t SectorError = 0U;
    uint32_t Index;
    HAL_StatusTypeDef Status;

    Record.Magic = BALL_IMPACT_CONFIG_MAGIC;
    Record.Version = BALL_IMPACT_CONFIG_VERSION;
    Record.StartLowerPulse = BallImpactConfig_StartLowerPulse;
    Record.StopRaisePulse = BallImpactConfig_StopRaisePulse;
    Record.Check = BallImpactConfig_Calculate_Check(&Record);

    Erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    Erase.Sector = BALL_IMPACT_CONFIG_FLASH_SECTOR;
    Erase.NbSectors = 1U;

    Status = HAL_FLASH_Unlock();
    if(Status == HAL_OK){
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP
                             | FLASH_FLAG_OPERR
                             | FLASH_FLAG_WRPERR
                             | FLASH_FLAG_PGAERR
                             | FLASH_FLAG_PGPERR
                             | FLASH_FLAG_PGSERR);
        Status = HAL_FLASHEx_Erase(&Erase, &SectorError);
    }

    Words = (const uint32_t *)&Record;
    for(Index = 0U;
        Status == HAL_OK && Index < sizeof(Record) / sizeof(uint32_t);
        Index++){
        Status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,
            BALL_IMPACT_CONFIG_FLASH_ADDRESS + Index * sizeof(uint32_t),
            Words[Index]);
    }
    (void)HAL_FLASH_Lock();

    SavedRecord = (const BallImpactConfig_FlashRecord_t *)
                  BALL_IMPACT_CONFIG_FLASH_ADDRESS;
    if(Status == HAL_OK
       && BallImpactConfig_Record_Is_Valid(SavedRecord)
       && SavedRecord->StartLowerPulse == Record.StartLowerPulse
       && SavedRecord->StopRaisePulse == Record.StopRaisePulse){
        BallImpactConfig_Dirty = 0U;
        BallImpactConfig_SaveError = 0U;
        return 1U;
    }

    BallImpactConfig_SaveError = 1U;
    return 0U;
}

uint16_t BallImpactConfig_Get_Start_Lower(void)
{
    return BallImpactConfig_StartLowerPulse;
}

uint16_t BallImpactConfig_Get_Stop_Raise(void)
{
    return BallImpactConfig_StopRaisePulse;
}

uint8_t BallImpactConfig_Is_Dirty(void)
{
    return BallImpactConfig_Dirty;
}

uint8_t BallImpactConfig_Has_Save_Error(void)
{
    return BallImpactConfig_SaveError;
}
