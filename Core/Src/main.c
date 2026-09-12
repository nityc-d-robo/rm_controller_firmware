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
#include <math.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MotorState motorstate[8] = {0};
float T_6 = 0.004;
float T_16 = 0.008;
Gain angle_gain = {.Kp = 0.6f, .Ki = 0.3f, .Kd = 0.0f};
Gain speed_gain = {.Kp = 10.0f, .Ki = 30.0f, .Kd = 0.0f}; // 10 //20 /0
volatile bool tim6 = 0;
volatile bool tim16 = 0;
int8_t motor_number = 8;
Sit motor_sit[8] = {0};
bool return_rpms = false;
HAL_StatusTypeDef status;
uint32_t TxMailbox;
uint8_t return_count = 0;
uint32_t reset_flags;

volatile uint8_t motor1_rx_data[8] = {0};
volatile uint32_t motor1_rx_id;
volatile bool motor1_rx_flag = false;

volatile uint8_t motor2_rx_data[8] = {0};
volatile uint32_t motor2_rx_id;
volatile bool motor2_rx_flag = false;

volatile uint8_t main1_rx_data[8] = {0};
volatile uint32_t main1_rx_id;
volatile bool main1_rx_flag = false;

volatile uint8_t main2_rx_data[8] = {0};
volatile uint32_t main2_rx_id;
volatile bool main2_rx_flag = false;

int8_t rm_id = 0;
// 割り込み内の処理を少なく、制御周期は優先度低く
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
void main1_rx(void);
void main2_rx(void);
void motor2_rx(void);
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
  HAL_Delay(100);
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
  gain_init();
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_FDCAN_GetErrorCounters(&hfdcan2, &ErrorCounters);
    live_tec = ErrorCounters.TxErrorCnt;
    live_rec = ErrorCounters.RxErrorCnt;
    // if ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0)
    //{
    //  HAL_FDCAN_Stop(&hfdcan2);
    //  HAL_FDCAN_Start(&hfdcan2);
    //}
    if (main1_rx_flag == true)
    {
      main1_rx_flag = false;
      main1_rx();
    }
    if (main2_rx_flag == true)
    {
      main2_rx_flag = false;
      main2_rx();
    }
    if (motor1_rx_flag == true)
    {
      motor1_rx_flag = false;
      // 処理なし
    }
    if (motor2_rx_flag == true)
    {
      motor2_rx_flag = false;
      motor2_rx();
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
}

void main1_rx(void)
{
  __disable_irq();
  int32_t rx_id = main1_rx_id;
  uint8_t rx_data[8] = {0};
  memcpy(rx_data, main1_rx_data, 8);
  __enable_irq();

  if (((rx_id >> 8) & 0x7) == 0x0)
  {
    // 緊急停止用
  }
  else if (((rx_id >> 3) & 0x03) == rm_id)
  {
    uint8_t motor_id = (rx_id & 0x07);
    uint8_t packet_type = rx_data[0];

    if (packet_type == 0x00)
    {
      motorstate[motor_id].speed.speed_pid_state.e_pre = 0;
      motorstate[motor_id].speed.speed_pid_state.ie = 0;
      motorstate[motor_id].motor_type = rx_data[1];
      motorstate[motor_id].mode = rx_data[2];
      if (rx_data[1] == 2)
      {
        FDCAN_TxHeaderTypeDef TxHeader;
        uint8_t tx_datas1[64] = {0};

        TxHeader.Identifier = (0x3 << 24) | (0x0 << 8) | motor_id; // 送信ID
        // TxHeader.Identifier = (0x3 << 24) | (0x0 << 8) | 0x7F; // 送信ID
        TxHeader.IdType = FDCAN_EXTENDED_ID;     // 標準ID
        TxHeader.TxFrameType = FDCAN_DATA_FRAME; // データフレーム
        TxHeader.DataLength = FDCAN_DLC_BYTES_8; // DLC
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker = 0;
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
        {
          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas1);
        }
        uint8_t tx_datas2[64] = {0};

        tx_datas2[0] = 1;
        TxHeader.Identifier = (0x6 << 24) | (0x0 << 8) | motor_id; // 送信ID
        TxHeader.IdType = FDCAN_EXTENDED_ID;                       // 標準ID
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;                   // データフレーム
        TxHeader.DataLength = FDCAN_DLC_BYTES_8;                   // DLC
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker = 0;
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
        {
          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas2);
        }
      }
    }
    else if (packet_type == 0x02)
    {
      uint16_t value1 = (rx_data[1] << 8) | rx_data[2];
      uint16_t value2 = (rx_data[3] << 8) | rx_data[4];
      uint16_t value3 = (rx_data[5] << 8) | rx_data[6];
      __fp16 half_value1;
      memcpy(&half_value1, &value1, 2);
      __fp16 half_value2;
      memcpy(&half_value2, &value2, 2);
      __fp16 half_value3;
      memcpy(&half_value3, &value3, 2);
      if (motorstate[motor_id].mode == SPEED)
      {
        motorstate[motor_id].speed.speed_gain.Kp = half_value1;
        motorstate[motor_id].speed.speed_gain.Ki = half_value2;
        motorstate[motor_id].speed.speed_gain.Kd = half_value3;
      }
      else if (motorstate[motor_id].mode == ANGLE)
      {
        motorstate[motor_id].angle.angle_gain.Kp = half_value1;
        motorstate[motor_id].angle.angle_gain.Ki = half_value2;
        motorstate[motor_id].angle.angle_gain.Kd = half_value3;
      }
    }
    else if (packet_type == 0x01)
    {
      if (motorstate[motor_id].motor_can_move < 1 && motorstate[motor_id].motor_type == 1)
      {
        // 無視
      }
      else
      {
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        if (motorstate[motor_id].mode == CURRENT)
        {
          motorstate[motor_id].target_current = (int16_t)((rx_data[1] << 8) | rx_data[2]);
        }
        else if (motorstate[motor_id].mode == SPEED)
        {
          motorstate[motor_id].main_can_move = 500; // 1s
          if (motorstate[motor_id].motor_type == 1)
          {
            motorstate[motor_id].speed.target_rpm = (int16_t)((rx_data[1] << 8) | rx_data[2]) * gear_ratio;
          }
          else
          {
            motorstate[motor_id].speed.target_rpm = (int16_t)((rx_data[1] << 8) | rx_data[2]);
          }
        }
        else if (motorstate[motor_id].mode == ANGLE)
        {
          uint16_t raw_target_angle = (rx_data[1] << 8) | rx_data[2];
          double pre_target_angle = motorstate[motor_id].angle.target_angle;
          __fp16 half_value;
          memcpy(&half_value, &raw_target_angle, 2);
          motorstate[motor_id].angle.target_angle = half_value;
          motorstate[motor_id].angle.half_target_angle = pre_target_angle - motorstate[motor_id].angle.target_angle;
        }
      }
    }
  }
}

void main2_rx(void)
{
  __disable_irq();
  int32_t rx_id = main2_rx_id;
  uint8_t rx_data[8] = {0};
  memcpy(rx_data, main2_rx_data, 8);
  __enable_irq();

  uint16_t type = rx_id >> 24 & 0x1F;
  if (type == 2)
  {
    uint16_t mode = rx_id >> 22 & 0x3;
    uint16_t fault = rx_id >> 16 & 0x3F;
    uint16_t motor_id = rx_id >> 8 & 0xFF;
    if (motor_id > 0)
    {
      if (fault == 0 && mode == 2)
      {
        motorstate[motor_id].motor_can_move = 250; // 0.5s
      }
    }
  }
}

void motor2_rx(void)
{
  __disable_irq();
  int32_t rx_id = motor2_rx_id;
  uint8_t rx_data[8] = {0};
  memcpy(rx_data, motor2_rx_data, 8);
  __enable_irq();

  if (rx_id > 0 && rx_id < 9)
  {
    uint16_t angle_raw = (rx_data[0] << 8) | rx_data[1];
    int16_t now_rpm = (rx_data[2] << 8) | rx_data[3];
    float new_rpm = now_rpm * alpha + motorstate[rx_id - 1].speed.rpm * (1 - alpha);
    motorstate[rx_id - 1].speed.rpm = new_rpm;
    motorstate[rx_id - 1].angle.raw_angle = (float)angle_raw * 360.0f / 8192.0f;
    motorstate[rx_id - 1].current = (rx_data[4] << 8) | rx_data[5];
    float diff = motorstate[rx_id - 1].angle.raw_angle - motorstate[rx_id - 1].angle.pre_raw_angle;
    if (diff < -300)
    {
      motorstate[rx_id - 1].resolution += 1;
    }
    else if (diff > 300)
    {
      motorstate[rx_id - 1].resolution -= 1;
    }
    motorstate[rx_id - 1].angle.angle = motorstate[rx_id - 1].angle.raw_angle + motorstate[rx_id - 1].resolution * 360;
    motorstate[rx_id - 1].angle.pre_raw_angle = motorstate[rx_id - 1].angle.raw_angle;
    if (motorstate[rx_id - 1].sub_sit == Stop)
    {
      motorstate[rx_id - 1].sub_sit = Move;
      motorstate[rx_id - 1].angle.angle_zero = motorstate[rx_id - 1].angle.angle;
    }
    motorstate[rx_id - 1].motor_can_move = 250; // 0.5s
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    motorstate[rx_id - 1].angle.angle = fmod(motorstate[rx_id - 1].angle.angle - motorstate[rx_id - 1].angle.angle_zero, 360 * gear_ratio);
    if (motorstate[rx_id - 1].angle.angle < 0)
    {
      motorstate[rx_id - 1].angle.angle += 360 * gear_ratio;
    }
  }
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
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
  }
}

void returnstates(void)
{
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
}

void speed_pid_task(void)
{

  int16_t current[8] = {0};
  int8_t rm_counter_1 = 0;
  int8_t rm_counter_2 = 0;

  for (int i = 0; i < 8; i++)
  {
    if (motorstate[i].motor_can_move > 0)
    {
      motorstate[i].motor_can_move -= 1;
    }
    else
    {
      motorstate[i].speed.speed_pid_state.e_pre = 0;
      motorstate[i].speed.speed_pid_state.ie = 0;
      motorstate[i].speed.rpm = 0;
      motorstate[i].speed.target_rpm = 0;
      motorstate[i].angle.angle_pid_state.e_pre = 0;
      motorstate[i].angle.angle_pid_state.ie = 0;
    }
    if (motorstate[i].mode == SPEED)
    {
      if (motorstate[i].main_can_move > 0)
      {
        motorstate[i].main_can_move -= 1;
      }
      else
      {
        motorstate[i].speed.target_rpm = 0;
      }
    }
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
      if (motorstate[i].motor_can_move > 0)
      {
        if (motorstate[i].mode == ANGLE)
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
        else if (motorstate[i].mode == SPEED)
        {
          if (return_rpms == true)
          {
            return_rpms = false;
          }
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
        else if (motorstate[i].mode == CURRENT && motorstate[i].motor_can_move > 1)
        {
          current[i] = motorstate[i].target_current;
          if (i > 3)
          {
            rm_counter_2 += 1;
          }
          else
          {
            rm_counter_1 += 1;
          }
        }
      }
    }
    else if (motorstate[i].motor_type == 2)
    {
      if (motorstate[i].motor_can_move > 1)
      {
        if (motorstate[i].mode == SPEED)
        {
          // ロボストライドの処理
          FDCAN_TxHeaderTypeDef TxHeader;
          uint8_t tx_datas[64] = {0};

          uint16_t torque = 32768; //-5.5Nm to 5.5Nm
          uint16_t motor_id = i;
          TxHeader.Identifier = (0x1 << 24) | (torque << 8) | motor_id; // 送信ID
          TxHeader.IdType = FDCAN_EXTENDED_ID;                          // 標準ID
          TxHeader.TxFrameType = FDCAN_DATA_FRAME;                      // データフレーム
          TxHeader.DataLength = FDCAN_DLC_BYTES_8;                      // DLC
          TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
          TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定５
          TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
          TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
          TxHeader.MessageMarker = 0;
          uint16_t target_angle = 0;
          float target_angular_velocity_float = (motorstate[i].speed.target_rpm * M_PI / 30.0f + 50.0f);
          uint32_t target_angular_velocity_32;
          if (target_angular_velocity_float > 100)
          {
            target_angular_velocity_32 = 6553500;
          }
          else if (target_angular_velocity_float < 0)
          {
            target_angular_velocity_32 = 0;
          }
          else
          {
            target_angular_velocity_32 = target_angular_velocity_float * 65535; //-50rad/s to 50rad/s
          }
          uint16_t target_angular_velocity = target_angular_velocity_32 / 100;
          uint16_t Kp = 0;    // 0 to 500
          uint16_t Kd = 2622; // 0 to 5

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
        }
        else if (motorstate[i].mode == ANGLE)
        {
          // ロボストライドの処理
          FDCAN_TxHeaderTypeDef TxHeader;
          uint8_t tx_datas[64] = {0};

          uint16_t torque = 32768; //-5.5Nm to 5.5Nm
          uint16_t motor_id = i;
          TxHeader.Identifier = (0x1 << 24) | (torque << 8) | motor_id; // 送信ID
          TxHeader.IdType = FDCAN_EXTENDED_ID;                          // 標準ID
          TxHeader.TxFrameType = FDCAN_DATA_FRAME;                      // データフレーム
          TxHeader.DataLength = FDCAN_DLC_BYTES_8;                      // DLC
          TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
          TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // BRS設定５
          TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Classic / FD
          TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
          TxHeader.MessageMarker = 0;
          float target_angle_float = ((motorstate[i].angle.target_angle) + 720);
          uint32_t target_angle_32;
          if (target_angle_float > 1440)
          {
            target_angle_32 = 1440 * 65535;
          }
          else if (target_angle_float < 0)
          {
            target_angle_32 = 0;
          }
          else
          {
            target_angle_32 = target_angle_float * 65535;
          }
          uint16_t target_angle = target_angle_32 / 1440;
          uint16_t target_angular_velocity = 32768; //-50rad/s to 50rad/s
          uint16_t Kp = 655;                        // 0 to 500
          uint16_t Kd = 2622;                       // 0 to 5

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
        }
        current[i] = 0;
      }
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
    if (motorstate[i].mode == ANGLE && motorstate[i].motor_type == 1)
    {
      motorstate[i].speed.target_rpm = (int16_t)angle_pid(i, (float)(motorstate[i].angle.target_angle * GEAR_RATIO - motorstate[i].angle.angle), &motorstate[i].angle.angle_pid_state.e_pre, &motorstate[i].angle.angle_pid_state.ie);
    }
  }
}

float speed_pid(int8_t motor_id, float e, float *e_pre, float *ie)
{
  float power;
  if (motorstate[motor_id].speed.target_rpm == 0)
  {
    *ie = 0;
    *e_pre = 0;
  }
  power = pid(motor_id, e, e_pre, ie, motorstate[motor_id].speed.speed_gain.Kp, motorstate[motor_id].speed.speed_gain.Ki, motorstate[motor_id].speed.speed_gain.Kd, T_6);
  if (*ie > MAX_SPEED_IE)
  {
    *ie = MAX_SPEED_IE;
  }
  else if (*ie < MAX_SPEED_IE * -1)
  {
    *ie = MAX_SPEED_IE * -1;
  }
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
  float rpm = pid(motor_id, e, e_pre, ie, motorstate[motor_id].angle.angle_gain.Kp, motorstate[motor_id].angle.angle_gain.Ki, motorstate[motor_id].angle.angle_gain.Kd, T_16);
  if (*ie > MAX_ANGLE_IE)
  {
    *ie = MAX_ANGLE_IE;
  }
  else if (*ie < MAX_ANGLE_IE * -1)
  {
    *ie = MAX_ANGLE_IE * -1;
  }
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
  *ie += (e + *e_pre) * t / 2;
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
