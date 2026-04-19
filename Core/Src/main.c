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
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MotorState motorstate[8] = {0};
float T = 0.001;
float Kp = 1.0f;
float Ki = 0.01f;
float Kd = 0.0f;
float view1 = 0;
float view2 = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_ErrorCountersTypeDef ErrorCounters;
uint32_t live_tec=0;
uint32_t live_rec=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  motorstate[1].target_rpm = 2000;
  motorstate[1].mode = SPEED;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_FDCAN_GetErrorCounters(&hfdcan2, &ErrorCounters);
    live_tec = ErrorCounters.TxErrorCnt;
    live_rec = ErrorCounters.RxErrorCnt;

    if ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0) {
    HAL_FDCAN_Stop(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan2);
    }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
double speed_pid(double e, double *e_pre, double *ie);
double angle_pid(double e, double *e_pre, double *ie);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // デバッグ
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_datas[64] = {0};

    TxHeader.Identifier = 0x200;             // 送信ID
    TxHeader.IdType = FDCAN_STANDARD_ID;     // 標準ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; // データフレーム
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // DLC
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    int16_t current[8] = {0};

    for (int i = 0; i < 8; i++)
    {
      if (motorstate[i].mode == ANGLE)
      {
        current[i] = (int16_t)angle_pid((double)(motorstate[i].target_angle - motorstate[i].angle), &motorstate[i].angle_pid_state.e_pre, &motorstate[i].angle_pid_state.ie);
      }
      else if (motorstate[i].mode == SPEED)
      {
        current[i] = (int16_t)speed_pid((double)(motorstate[i].target_rpm - motorstate[i].rpm), &motorstate[i].speed_pid_state.e_pre, &motorstate[i].speed_pid_state.ie);
      }
      else
      {
      }
    }
    view1 = current[1];
    tx_datas[0] = 0;
    tx_datas[1] = 0;
    tx_datas[2] = (current[1] >> 8);
    tx_datas[3] = (current[1] & 0xFF);
    tx_datas[4] = 0;
    tx_datas[5] = 0;
    tx_datas[6] = 0;
    tx_datas[7] = 0;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
    {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas);
    }
  }
}

double speed_pid(double e, double *e_pre, double *ie) {
  double de = (e - *e_pre) / T;
  *ie = *ie + (e + *e_pre) * T / 2;
  if (*ie > 10000)
  {
    *ie = 10000;
  }
  else if (*ie < -10000)
  {
    *ie = -10000;
  }
  double P = e * Kp;
  double I = *ie * Ki;
  double D = de * Kd;
  int16_t power = P + I + D;
  if (power > 10000)
  {
    power = 10000;
  }
  else if (power < -10000)
  {
    power = -10000;
  }
  *e_pre = e;
  return power;
}

double angle_pid(double e, double *e_pre, double *ie)
{
  if (e > 180)
  {
    e -= 360;
  }
  else if (e < -180)
  {
    e += 360;
  }
  double de = (e - *e_pre) / T;
  *ie = *ie + (e + *e_pre) * T / 2;
  if (*ie > 10000)
  {
    *ie = 10000;
  }
  else if (*ie < -10000)
  {
    *ie = -10000;
  }
  double P = e * Kp;
  double I = *ie * Ki;
  double D = de * Kd;
  int16_t power = P + I + D;
  if (power > 10000)
  {
    power = 10000;
  }
  else if (power < -10000)
  {
    power = -10000;
  }
  *e_pre = e;
  return power;
}

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
