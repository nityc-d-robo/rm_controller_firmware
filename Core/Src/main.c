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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MotorState motorstate[8] = {0};
float T = 0.001;
Gain angle_gain = {Kp: 0.5f , Ki:0.2f, Kd: 0.2f};
Gain speed_gain = {Kp: 1.5f , Ki:1.0f , Kd: 0.001f};
float view1 = 0;
float view2 = 0;
Sit motor_sit[8] = { 0 };
float ε = 100;
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
  motorstate[1].target_rpm = 500;
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
double speed_pid(int8_t motor_id,double e, double *e_pre, double *ie);
double angle_pid(int8_t motor_id,double e, double *e_pre, double *ie);

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
      if (motorstate[i].rpm == 0) {
        motor_sit[i] = Stop;
      } else {
        motor_sit[i] = Move;
      }
      if (motorstate[i].mode == ANGLE)
      {
        current[i] = (int16_t)angle_pid(i,(double)(motorstate[i].target_angle * 19.204 - motorstate[i].angle), &motorstate[i].angle_pid_state.e_pre, &motorstate[i].angle_pid_state.ie);
      }
      else if (motorstate[i].mode == SPEED)
      {
        current[i] = (int16_t)speed_pid(i,(double)(motorstate[i].target_rpm - motorstate[i].rpm), &motorstate[i].speed_pid_state.e_pre, &motorstate[i].speed_pid_state.ie);
      }
      else
      {
      }
    }
    view1 = current[1];
    view2 = motorstate[1].rpm / 19;
    tx_datas[0] = (current[0] >> 8);
    tx_datas[1] = (current[0] & 0xFF);
    tx_datas[2] = (current[1] >> 8);
    tx_datas[3] = (current[1] & 0xFF);
    tx_datas[4] = (current[2] >> 8);
    tx_datas[5] = (current[2] & 0xFF);
    tx_datas[6] = (current[3] >> 8);
    tx_datas[7] = (current[3] & 0xFF);
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
    {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas);
    }
  }
}

double speed_pid(int8_t motor_id,double e, double *e_pre, double *ie) {
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
  double P = e * speed_gain.Kp;
  double I = *ie * speed_gain.Ki;
  double D = de * speed_gain.Kd;
  int16_t power = P + I + D;
  if (power > 1000)
  {
    power = 1000;
  }
  else if (power < -1000)
  {
    power = -1000;
  }
  *e_pre = e;
  if (motor_sit[motor_id] == Stop) { 
    power = 1000; 
    motor_sit[motor_id] = Move;
  }
  return power;
}

double angle_pid(int8_t motor_id,double e, double *e_pre, double *ie)
{
  if (motor_sit[motor_id] == Stop) {
    motor_sit[motor_id] = Move;
  }
  if (e > 3457)
  {
    e -= 6913;
  }
  else if (e < -3457)
  {
    e += 6913;
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
  double P = e * angle_gain.Kp;
  double I = *ie * angle_gain.Ki;
  double D = de * angle_gain.Kd;
  int16_t power = P + I + D;
  if (abs(e) > ε) {
    if (e < 0) {
      power -= 500;
    } else if ( e > 0 ) {
      power += 500;
    }
  }
  if (power > 10000)
  {
    power = 10000;
  }
  else if (power < -10000)
  {
    power = -10000;
  }
  *e_pre = e;
  if (motor_sit[motor_id] == Stop) { 
    power = 1000;
    motor_sit[motor_id] = Move;
  }
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
