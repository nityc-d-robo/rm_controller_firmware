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
#include "math.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MotorState motorstate[8] = {0};
float T_6 = 0.002;
float T_16 = 0.004;
Gain angle_gain = {.Kp = 0.6f, .Ki = 0.3f, .Kd = 0.0f};
Gain speed_gain = {.Kp = 10.0f, .Ki = 20.0f, .Kd = 0.0f};
float view1 = 0;
float view2 = 0;
float view3 = 0;
float view4 = 0;
volatile bool tim6 = 0;
volatile bool tim16 = 0;
int8_t motor_number = 8;
Sit motor_sit[8] = {0};
bool return_rpms = false;
HAL_StatusTypeDef status;
uint32_t TxMailbox;
uint8_t return_count = 0;
// float ε = 20;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_ErrorCountersTypeDef ErrorCounters;
uint32_t live_tec = 0;
uint32_t live_rec = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void speed_pid_task(void);
void angle_pid_task(void);
void gain_init();
float speed_pid(int8_t motor_id, float e, float *e_pre, float *ie);
float angle_pid(int8_t motor_id, float e, float *e_pre, float *ie);
float pid(int8_t motor_id, float e, float *e_pre, float *ie, float p_gain, float i_gain, float d_gain, float t);
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
  motorstate[0].speed.target_rpm = 0;
  motorstate[0].mode = INIT;
  gain_init();
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_FDCAN_GetErrorCounters(&hfdcan2, &ErrorCounters);
    //  live_tec = ErrorCounters.TxErrorCnt;
    //  live_rec = ErrorCounters.RxErrorCnt;
    if ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0)
    {
      HAL_FDCAN_Stop(&hfdcan2);
      HAL_FDCAN_Start(&hfdcan2);
    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void gain_init()
{
  for (int i = 0; i < 8; i++)
  {
    motorstate[i].speed.speed_gain = speed_gain;
    motorstate[i].angle.angle_gain = angle_gain;
    motorstate[i].motor_type = 0;
  }
  //  FDCAN_TxHeaderTypeDef TxHeader;
  //  uint8_t tx_datas[64] = {0};

  //  TxHeader.Identifier = 0x3 | 0x0;         // 送信ID
  //  TxHeader.IdType = FDCAN_EXTENDED_ID;     // 標準ID
  //  TxHeader.TxFrameType = FDCAN_DATA_FRAME; // データフレーム
  //  TxHeader.DataLength = FDCAN_DLC_BYTES_8; // DLC
  //  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  //  TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
  //  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
  //  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  //  TxHeader.MessageMarker = 0;
  //  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
  //  {
  //    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas);
  //  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    speed_pid_task();
  }
  if (htim->Instance == TIM16)
  {
    angle_pid_task();
  }
}

void returnstates(void)
{
  /*
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t tx_datas[64] = {0};

  for (int i = 0; i < motor_number; i++)
  {
    TxHeader.Identifier = 0x260 | i;         // 送信ID
    TxHeader.IdType = FDCAN_STANDARD_ID;     // 標準ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; // データフレーム
    TxHeader.DataLength = FDCAN_DLC_BYTES_6; // DLC
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    tx_datas[0] = (motorstate[i].current >> 8);
    tx_datas[1] = (motorstate[i].current & 0xFF);
    tx_datas[2] = (motorstate[i].speed.rpm >> 8);
    tx_datas[3] = (motorstate[i].speed.rpm & 0xFF);
    tx_datas[4] = ((int16_t)motorstate[i].angle.angle >> 8);
    tx_datas[5] = ((int16_t)motorstate[i].angle.angle & 0xFF);
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
    {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_datas);
    }
  }
  */
}

void speed_pid_task(void)
{
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // デバッグ

  int16_t current[8] = {0};
  int8_t rm_counter_1 = 0;
  int8_t rm_counter_2 = 0;

  for (int i = 0; i < 8; i++)
  {
    if (motorstate[i].motor_type == 1)
    {
      if (motorstate[i].speed.rpm == 0)
      {
        motor_sit[i] = Stop;
      }
      else
      {
        motor_sit[i] = Move;
      }
      if (motorstate[i].mode == ANGLE && motorstate[i].sub_sit == Move)
      {
        current[i] = (int16_t)speed_pid(i, (float)(motorstate[i].speed.target_rpm - motorstate[i].speed.rpm), &motorstate[i].speed.speed_pid_state.e_pre, &motorstate[i].speed.speed_pid_state.ie);
        if (i > 3)
        {
          rm_counter_2 += 1;
        }
        else
        {
          rm_counter_1 += 1;
        }
      }
      else if (motorstate[i].mode == SPEED && motorstate[i].sub_sit == Move)
      {
        // if (return_count >= 100) {
        if (return_rpms == true)
        {
          //        returnrpms();
          return_rpms = false;
          //  return_count = 0;
        }
        // return_count += 1;
        current[i] = (int16_t)speed_pid(i, (float)(motorstate[i].speed.target_rpm - motorstate[i].speed.rpm), &motorstate[i].speed.speed_pid_state.e_pre, &motorstate[i].speed.speed_pid_state.ie);
        if (i > 4)
        {
          rm_counter_2 += 1;
        }
        else
        {
          rm_counter_1 += 1;
        }
      }
      else if (motorstate[i].mode == CURRENT && motorstate[i].sub_sit == Move)
      {
        current[i] = motorstate[i].current;
        if (i > 4)
        {
          rm_counter_2 += 1;
        }
        else
        {
          rm_counter_1 += 1;
        }
      }
    }
    else if (motorstate[i].motor_type == 2)
    {
      /* ロボストライドの処理
      FDCAN_TxHeaderTypeDef TxHeader;
      uint8_t tx_datas[64] = {0};

      uint16_t torque = 65535;
      TxHeader.Identifier = (0x1 << 24) | (torque << 8) | (0x01); // 送信ID
      TxHeader.IdType = FDCAN_EXTENDED_ID;                        // 標準ID
      TxHeader.TxFrameType = FDCAN_DATA_FRAME;                    // データフレーム
      TxHeader.DataLength = FDCAN_DLC_BYTES_8;                    // DLC
      TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
      TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
      TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
      TxHeader.MessageMarker = 0;
      uint16_t target_angle = 65535;
      uint16_t target_angular_velocity = 65535;
      uint16_t Kp = 65535;
      uint16_t Kd = 65535;

      tx_datas[0] = (target_angle >> 8);
      tx_datas[1] = (target_angle & 0xFF);
      tx_datas[2] = (target_angular_velocity >> 8);
      tx_datas[3] = (target_angular_velocity & 0xFF);
      tx_datas[4] = (Kp >> 8);
      tx_datas[5] = (Kp & 0xFF);
      tx_datas[6] = (Kd >> 8);
      tx_datas[7] = (Kd & 0xFF);
      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
      {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas);
      }
      current[i] = 0;
      */
    }
  }
  if (rm_counter_1 > 0)
  {
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
    view1 = current[0];
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
  if (rm_counter_2 > 0)
  {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_datas[64] = {0};
    TxHeader.Identifier = 0x1FF;             // 送信ID
    TxHeader.IdType = FDCAN_STANDARD_ID;     // 標準ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; // データフレーム
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // DLC
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    view2 = current[5];
    tx_datas[0] = (current[4] >> 8);
    tx_datas[1] = (current[4] & 0xFF);
    tx_datas[2] = (current[5] >> 8);
    tx_datas[3] = (current[5] & 0xFF);
    tx_datas[4] = (current[6] >> 8);
    tx_datas[5] = (current[6] & 0xFF);
    tx_datas[6] = (current[7] >> 8);
    tx_datas[7] = (current[7] & 0xFF);
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
    {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas);
    }
  }
}

void angle_pid_task(void)
{
  for (int i = 0; i < 8; i++)
  {
    if (motorstate[i].mode == ANGLE)
    {
      motorstate[i].speed.target_rpm = (int16_t)angle_pid(i, (float)(motorstate[i].angle.target_angle * GEAR_RATIO - motorstate[i].angle.angle), &motorstate[i].angle.angle_pid_state.e_pre, &motorstate[i].angle.angle_pid_state.ie);
    }
  }
}

float speed_pid(int8_t motor_id, float e, float *e_pre, float *ie)
{
  if (*ie > MAX_SPEED_IE)
  {
    *ie = MAX_SPEED_IE;
  }
  else if (*ie < MAX_SPEED_IE * -1)
  {
    *ie = MAX_SPEED_IE * -1;
  }
  int16_t power = pid(motor_id, e, e_pre, ie, motorstate[motor_id].speed.speed_gain.Kp, motorstate[motor_id].speed.speed_gain.Ki, motorstate[motor_id].speed.speed_gain.Kd, T_6);
  if (power > MAX_POWER)
  {
    power = MAX_POWER;
  }
  else if (power < MAX_POWER * -1)
  {
    power = MAX_POWER * -1;
  }
  *e_pre = e;
  return power;
}

float angle_pid(int8_t motor_id, float e, float *e_pre, float *ie)
{
  if (motor_sit[motor_id] == Stop)
  {
    motor_sit[motor_id] = Move;
  }
  if (e > RESOLUTION / 2)
  {
    e -= RESOLUTION;
  }
  else if (e < RESOLUTION / -2)
  {
    e += RESOLUTION;
  }
  if (*ie > MAX_ANGLE_IE)
  {
    *ie = MAX_ANGLE_IE;
  }
  else if (*ie < MAX_ANGLE_IE * -1)
  {
    *ie = MAX_ANGLE_IE * -1;
  }
  int16_t rpm = pid(motor_id, e, e_pre, ie, motorstate[motor_id].angle.angle_gain.Kp, motorstate[motor_id].angle.angle_gain.Ki, motorstate[motor_id].angle.angle_gain.Kd, T_16);
  if (rpm > MAX_RPM)
  {
    rpm = MAX_RPM;
  }
  else if (rpm < MAX_RPM * -1)
  {
    rpm = MAX_RPM * -1;
  }
  *e_pre = e;
  return rpm;
}

float pid(int8_t motor_id, float e, float *e_pre, float *ie, float p_gain, float i_gain, float d_gain, float t)
{
  float de = (e - *e_pre) / t;
  if (fabsf(e) < I_MAX_ERROR && fabsf(e) > I_MIN_ERROR)
  {
    *ie = *ie + (e + *e_pre) * t / 2;
  }
  float P = e * p_gain;
  float I = *ie * i_gain;
  float D = de * d_gain;
  float power = P + I + D;
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
