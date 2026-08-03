/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "Timer.h"
#include "Key_manage.h"
#include "OLED.h"
#include "Task_Cloud.h"
#include "K230.h"
#include "DO_Device.h"
#include "Task_Ball_Contral.h"
#include "JY61P.h"
#include "Serial.h"
#include "Task_Blue.h"
#include "Ball_Impact_Config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Timer_t Timer_Tick;
uint8_t i = 0;
uint8_t j = 0;
extern DO_Device_t Laser;
Serial_t Serial_JY61P;
JY61P_CalibratedAccel_t Accel_Calibrated;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Timer_Tick_Callback(Timer_t *timer){
  if(timer == &Timer_Tick){
    j++;
    Key_Global_Tick();
    Task_Ball_Contral_Tick();
  }
}

static void Ball_Impact_Config_OLED_Update(void){
  static uint16_t Last_Start_Lower = 0xFFFFU;
  static uint16_t Last_Stop_Raise = 0xFFFFU;
  static uint8_t Last_Dirty = 0xFFU;
  static uint8_t Last_Save_Error = 0xFFU;
  uint16_t Start_Lower = BallImpactConfig_Get_Start_Lower();
  uint16_t Stop_Raise = BallImpactConfig_Get_Stop_Raise();
  uint8_t Dirty = BallImpactConfig_Is_Dirty();
  uint8_t Save_Error = BallImpactConfig_Has_Save_Error();

  /* 只有参数或保存状态变化时才刷新，避免OLED通信拖慢控球循环。 */
  if(Start_Lower == Last_Start_Lower
     && Stop_Raise == Last_Stop_Raise
     && Dirty == Last_Dirty
     && Save_Error == Last_Save_Error){
    return;
  }

  OLED_ShowString(1, 1, "K2 LOW  K3 RAISE");
  if(Save_Error){
    OLED_ShowString(2, 1, "K4 SAVE:ERROR   ");
  }
  else if(Dirty){
    OLED_ShowString(2, 1, "K4 SAVE:PENDING ");
  }
  else{
    OLED_ShowString(2, 1, "K4 SAVE:OK      ");
  }
  OLED_ShowString(3, 1, "LOWER:          ");
  OLED_ShowNum(3, 8, Start_Lower, 3);
  OLED_ShowString(4, 1, "RAISE:          ");
  OLED_ShowNum(4, 8, Stop_Raise, 3);

  Last_Start_Lower = Start_Lower;
  Last_Stop_Raise = Stop_Raise;
  Last_Dirty = Dirty;
  Last_Save_Error = Save_Error;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM14_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  BallImpactConfig_Init();
  Key_Glabal_Init();
  OLED_Init();
  Ball_Impact_Config_OLED_Update();
  Task_Ball_Contral_Init();
  Task_Ball_Contral_Pop_Init();
  Serial_Init(&Serial_JY61P, Serial_5);
  JY61P_Init(&Serial_JY61P);
  Task_Blue_Init();
  Timer_Init(&Timer_Tick, Timer_14);
  Timer_Start_IT(&Timer_Tick, Timer_Tick_Callback);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Key_Global_Trigger_Event();

    if(JY61P_Accel_Update() && JY61P_Is_Calibrated()){
      JY61P_Get_Calibrated_Accel(&Accel_Calibrated);
      Task_Ball_Contral_Update_Acceleration(Accel_Calibrated.Ay);
    }

    Task_Ball_Contral_Loop();
    Task_Blue_Loop();
    Ball_Impact_Config_OLED_Update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
