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
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdlib.h"
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
MotorState motorstate[8] = {0};
volatile RxDatas rx_datas;
int16_t can1_reset_cooltime = 0;
int16_t can2_reset_cooltime = 0;
volatile int16_t main_can_move = 0; //mainとcan通信ができているか
uint8_t rm_id = 0;
Gain rm_angle_gain = {.Kp = 0.6f, .Ki = 0.3f, .Kd = 0.0f};
Gain rm_speed_gain = {.Kp = 2.0f, .Ki = 2.0f, .Kd = 0.0f};
Gain rs_angle_gain = {.Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f};
volatile int16_t view1 = 0;
volatile int16_t view2 = 0;
volatile int16_t view3 = 0;
volatile int16_t view4 = 0;
volatile int16_t view5 = 0;
uint32_t t_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void main_rx(uint8_t motor_id);
void rm_rx(uint8_t motor_id);
void rs_rx(uint8_t motor_id);
void rs_init(uint8_t motor_id);
void can_send(uint32_t id, uint8_t *tx_data,uint8_t id_type,uint8_t channel);
void speed_pid_task();
void angle_pid_task();
void countdown();
void gain_init(uint8_t motor_id,int8_t motor_type);
int16_t rm_move(int i,bool *rm_send1,bool *rm_send2);
void rs_move(int i);
float pid(int i, Mode mode);
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
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //異常時にCANをリセット
    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0 && can1_reset_cooltime == 0)
    {
      HAL_FDCAN_Stop(&hfdcan1);
      HAL_FDCAN_Start(&hfdcan1);
      can1_reset_cooltime = 25;
    }
    if ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0 && can2_reset_cooltime == 0)
    {
      HAL_FDCAN_Stop(&hfdcan2);
      HAL_FDCAN_Start(&hfdcan2);
      can2_reset_cooltime = 25;
    }
    //CAN受信の処理
    for (int i = 0; i < 8; i++) {
      //メインからの受信
      if (rx_datas.main_rx[i].rx_flag == true) {
        rx_datas.main_rx[i].rx_flag = false;
        main_rx(i);
      }
      //ロボマスからの受信
      if (rx_datas.rm_rx[i].rx_flag == true) {
        rx_datas.rm_rx[i].rx_flag = false;
        rm_rx(i);
      }
      //ロボストライドからの受信
      if (rx_datas.rs_rx[i].rx_flag == true) {
        rx_datas.rs_rx[i].rx_flag = false;
        rs_rx(i);
      }
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

void can_send(uint32_t id,uint8_t *tx_data,uint8_t id_type,uint8_t channel) {
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t tx_datas[8] = {0};
  if (id_type == 0) {
    TxHeader.Identifier = id;             // 送信ID
    TxHeader.IdType = FDCAN_STANDARD_ID;  // 標準ID
  } else {
    TxHeader.Identifier = id;             // 送信ID
    TxHeader.IdType = FDCAN_EXTENDED_ID;  // 拡張ID
  }
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;            // データフレーム
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;            // DLC
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;             // BRS設定５
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;              // Classic
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  memcpy(tx_datas,tx_data,8);
  
  if (channel == 0) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
    {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_datas);
    }
  } else {
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
    {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_datas);
    }
  }
}

void gain_init(uint8_t motor_id,int8_t motor_type) {
  if (motor_type == 1) {
    motorstate[motor_id].speed.speed_gain = rm_speed_gain;
    motorstate[motor_id].angle.angle_gain = rm_angle_gain;
  } else if (motor_type == 2) {
    motorstate[motor_id].angle.angle_gain = rs_angle_gain;
  }
}

void rs_init(uint8_t motor_id) {
  uint8_t tx_datas[8] = {0};
  can_send((0x3 << 24) | (0x0 << 8) | motor_id,tx_datas,1,1);
  tx_datas[0] = 1;
  can_send((0x6 << 24) | (0x0 << 8) | motor_id,tx_datas,1,1);
}

void main_rx(uint8_t motor_id) {
  __disable_irq();
  uint8_t rx_data[8] = {0};
  uint32_t rx_id = rx_datas.main_rx[motor_id].rx_id;
  memcpy(rx_data,rx_datas.main_rx[motor_id].rx_data, 8);
  __enable_irq();
  uint8_t packet_type = rx_data[0];

  if (packet_type == 0x00) {
    //init処理
    motorstate[motor_id].speed.speed_pid_state.e_pre = 0;
    motorstate[motor_id].speed.speed_pid_state.ie = 0;
    motorstate[motor_id].motor_type = rx_data[1];
    motorstate[motor_id].mode = rx_data[2];
    gain_init(motor_id,motorstate[motor_id].motor_type);
    if (rx_data[1] == 2) {
      rs_init(motor_id);   //ロボストライド用のinit処理
    }
  } else if (packet_type == 0x01) {
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    //値を受け取る処理
    if (motorstate[motor_id].motor_can_move < 1) {
      return;
    }
    main_can_move = 125;
    if (motorstate[motor_id].mode == CURRENT) {
      motorstate[motor_id].current.target_current = (int16_t)((rx_data[1] << 8) | rx_data[2]);
    } else if (motorstate[motor_id].mode == SPEED) {
      if (motorstate[motor_id].motor_type == 1) {
        motorstate[motor_id].speed.target_rpm = (int16_t)((rx_data[1] << 8) | rx_data[2]) * GEAR_RATIO;     //もしロボマスなら、ギア比をかける
      } else {
        motorstate[motor_id].speed.target_rpm = (int16_t)((rx_data[1] << 8) | rx_data[2]);
      }
    } else if (motorstate[motor_id].mode == ANGLE) {
      uint16_t raw_target_angle = (rx_data[1] << 8) | rx_data[2];
      __fp16 half_value;
      memcpy(&half_value,&raw_target_angle,2);
      if (motorstate[motor_id].motor_type == 1) {
        motorstate[motor_id].angle.target_angle = half_value * GEAR_RATIO;     //もしロボマスなら、ギア比をかける
      } else {
        motorstate[motor_id].angle.target_angle = half_value;
      }
    }
  } else if (packet_type == 0x02) {
    //ゲインを変更
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
}

void rm_rx(uint8_t motor_id) {
  __disable_irq();
  uint8_t rx_data[8] = {0};
  uint32_t rx_id = rx_datas.rm_rx[motor_id].rx_id;
  memcpy(rx_data,rx_datas.rm_rx[motor_id].rx_data, 8);
  __enable_irq();

  //currentを取得
  int16_t now_current = (rx_data[4] << 8) | rx_data[5];
  float new_current = now_current * alpha + motorstate[motor_id].current.current * (1 - alpha);
  motorstate[motor_id].current.current = (int16_t)new_current;
  
  //speedを取得
  int16_t now_rpm = (rx_data[2] << 8) | rx_data[3];
  float new_rpm = now_rpm * alpha + motorstate[motor_id].speed.rpm * (1 - alpha);
  motorstate[motor_id].speed.rpm = (int16_t)new_rpm;

  //angleを取得
  uint16_t angle_raw = (rx_data[0] << 8) | rx_data[1];
  float angle_360 = (float)angle_raw * 360.0f / 8192.0f;
  float diff = angle_360 - motorstate[motor_id].angle.pre_angle_360;
  if (diff < -300) {
    motorstate[motor_id].angle.resolution += 1;
  } else if (diff > 300) {
    motorstate[motor_id].angle.resolution -= 1;
  }
  motorstate[motor_id].angle.angle = angle_360 + motorstate[motor_id].angle.resolution * 360;
  motorstate[motor_id].angle.pre_angle_360 = angle_360;
  if (motorstate[motor_id].first_move == false) {
    motorstate[motor_id].first_move = true;
    motorstate[motor_id].angle.angle_zero = motorstate[motor_id].angle.angle;
  }
  motorstate[motor_id].motor_can_move = 125;
  motorstate[motor_id].angle.angle = fmod(motorstate[motor_id].angle.angle - motorstate[motor_id].angle.angle_zero,360 * GEAR_RATIO);
  if (motorstate[motor_id].angle.angle < 0) {
    motorstate[motor_id].angle.angle += 360 * GEAR_RATIO;
  }
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
}

void rs_rx(uint8_t motor_id) {
  __disable_irq();
  uint8_t rx_data[8] = {0};
  uint32_t rx_id = rx_datas.rs_rx[motor_id].rx_id;
  memcpy(rx_data,rx_datas.rs_rx[motor_id].rx_data, 8);
  __enable_irq();
  uint16_t type = rx_id >> 24 & 0x1F;
  if (type == 2) {
    uint16_t mode = rx_id >> 22 & 0x3;
    uint16_t fault = rx_id>> 16 & 0x3F;
    if (fault == 0 && mode == 2) {
      //motor_canが動いていることを確認
      motorstate[motor_id].motor_can_move = 125;
      //currentを取得
      uint16_t current_raw = (rx_data[4] << 8 ) | rx_data[5];
      int32_t current = current_raw * 20000 / 65535 - 10000; //電流値は10000~-10000
      motorstate[motor_id].current.current = (int16_t)current;
      //speedを取得
      uint16_t speed_raw = (rx_data[2] << 8) | rx_data[3];
      float speed = (float)speed_raw * 100.0f / 65535.0f - 50.0f;
      motorstate[motor_id].speed.rpm = (int16_t)(speed * 30.0f / (float)M_PI);
      //angleを取得
      uint16_t angle_raw = (rx_data[0] << 8) | rx_data[1];
      float angle_360 = angle_raw / 65535.0f * 1440.0f - 720.0f;
      float diff = angle_360 - motorstate[motor_id].angle.pre_angle_360;
      if (diff < -720) {
        motorstate[motor_id].angle.resolution += 1;
      } else if (diff > 720) {
        motorstate[motor_id].angle.resolution -= 1;
      }
      motorstate[motor_id].angle.pre_angle_360 = angle_360;
      motorstate[motor_id].angle.angle = angle_360 + motorstate[motor_id].angle.resolution * 1440.0f;
    }
  }
}

void countdown() {
  //リセットのクールタイムをカウントダウン
  if (can1_reset_cooltime > 0)
  {
    can1_reset_cooltime -= 1;
  }
  if (can2_reset_cooltime > 0)
  {
    can2_reset_cooltime -= 1;
  }
  //main_can_moveをカウントダウン
  if (main_can_move > 0) {
    main_can_move -= 1;
  } else {
    //main_canが動いていない時
    for (int i = 0; i < 8; i++) 
    {
      if (motorstate[i].mode == SPEED || motorstate[i].mode == CURRENT) {
        motorstate[i].speed.target_rpm = 0;
        motorstate[i].current.target_current = 0;
      }
    }
  }
  //motor_can_moveをカウントダウン
  for (int i = 0; i < 8; i++) {
    if (motorstate[i].motor_can_move > 0) {
      motorstate[i].motor_can_move -= 1;
    } else {
      //motor_canが動いていない時
      motorstate[i].current.target_current = 0;
      motorstate[i].speed.speed_pid_state.e_pre = 0;
      motorstate[i].speed.speed_pid_state.ie = 0;
      motorstate[i].speed.target_rpm = 0;
      motorstate[i].speed.rpm = 0;
      motorstate[i].angle.angle_pid_state.e_pre = 0;
      motorstate[i].angle.angle_pid_state.ie = 0;
    }
  }
}

float pid(int i,Mode mode) {
  float *ie = 0;
  float e = 0;
  float *e_pre = 0;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float t = 0;
  float power = 0;
  int16_t max_power = 0;
  int16_t min_power = 0;
  if (mode == SPEED) {
    e = motorstate[i].speed.target_rpm - motorstate[i].speed.rpm;
    ie = &motorstate[i].speed.speed_pid_state.ie;
    e_pre = &motorstate[i].speed.speed_pid_state.e_pre;
    Kp = motorstate[i].speed.speed_gain.Kp;
    Ki = motorstate[i].speed.speed_gain.Ki;
    Kd = motorstate[i].speed.speed_gain.Kd;
    max_power = MAX_POWER;
    min_power = MIN_POWER;
    t = T_6;
  } else if (mode == ANGLE) {
    e = motorstate[i].angle.target_angle - motorstate[i].angle.angle;
    if (motorstate[i].motor_type == 1) {
      if (e > RESOLUTION / 2) 
      {
        e -= RESOLUTION;
      } else if (e < RESOLUTION / -2) {
        e += RESOLUTION;
      }
      max_power = RM_MAX_RPM;
      min_power = RM_MIN_RPM;
    } else if (motorstate[i].motor_type == 2) {
      max_power = RS_MAX_RPM;
      min_power = RS_MIN_RPM;
    }
    ie = &motorstate[i].angle.angle_pid_state.ie;
    e_pre = &motorstate[i].angle.angle_pid_state.e_pre;
    Kp = motorstate[i].angle.angle_gain.Kp;
    Ki = motorstate[i].angle.angle_gain.Ki;
    Kd = motorstate[i].angle.angle_gain.Kd;
    t = T_16;
  }

  float de = (e - *e_pre) / t;
  if (*ie > MAX_IE && (e + *e_pre) > 0) {
    //ieを増やさない
  } else if (*ie < MIN_IE && (e + *e_pre) < 0) {
    //ieを減らさない
  }else {
    *ie += (e + *e_pre) * t / 2;
  }
  float P = e * Kp;
  float I = *ie * Ki;
  float D = de * Kd;
  power = P + I + D;
  if (power > max_power) {
    power = max_power;
  } else if (power < min_power) {
    power = min_power;
  }
  *e_pre = e;
  return power;
}

int16_t rm_move(int i,bool *rm_send1,bool *rm_send2) {
  int16_t current;
  float power = 0;
  if (motorstate[i].motor_can_move > 0) {
    if (motorstate[i].mode == ANGLE || motorstate[i].mode == SPEED) {
      //ロボマスを動かすか確認
      if (i > 3) {
        if (!*rm_send2) {
          *rm_send2 = true;
        }
      } else {
        if (!*rm_send1) {
          *rm_send1 = true;
        }
      }
      //目標速度0のときie/e_preをリセット
      if (motorstate[i].speed.target_rpm == 0) {
        motorstate[i].speed.speed_pid_state.e_pre = 0;
        motorstate[i].speed.speed_pid_state.ie = 0;
      }
      power = pid(i,SPEED);
    } else if (motorstate[i].mode == CURRENT) {
      power = motorstate[i].current.target_current;
      //ロボマスを動かすか確認
      if (i > 3) {
        if (!*rm_send2) {
          *rm_send2 = true;
        }
      } else {
        if (!*rm_send1) {
          *rm_send1 = true;
        }
      }
    }
  }
  current = (int16_t)power;
  return current;
}

void rs_move(int i) {
  if (motorstate[i].motor_can_move == 0) {
    return;
  }
  uint16_t torque = 32768;
  uint32_t id = (0x1 << 24) | (torque << 8) | i;
  uint8_t tx_data[8] = {0};

  float target_speed = motorstate[i].speed.target_rpm / 30.0f * (float)M_PI;
  if (target_speed > MAX_RAD_S) {
    target_speed = MAX_RAD_S;
  } else if (target_speed < MIN_RAD_S) {
    target_speed = MIN_RAD_S;
  }
  float target_angular_velocity_float = (target_speed + 50.0f) * 65535.0f / 100.0f;
  uint16_t target_angular_velocity = (uint16_t)target_angular_velocity_float;
  uint16_t target_angle = 0;
  uint16_t Kp = 0;
  uint16_t Kd = 2622;
  tx_data[0] = (target_angle >> 8);
  tx_data[1] = (target_angle & 0xFF);
  tx_data[2] = (target_angular_velocity >> 8);
  tx_data[3] = (target_angular_velocity & 0xFF);
  tx_data[4] = (Kp >> 8);
  tx_data[5] = (Kp & 0xFF);
  tx_data[6] = (Kd >> 8);
  tx_data[7] = (Kd & 0xFF);
  can_send(id,tx_data,1,1);
}

void speed_pid_task(void) {
  bool rm_send1 = false;
  bool rm_send2 = false;
  uint8_t tx_data[8] = {0};
  int16_t current[8] = {0};
  for (int i = 0; i < 8; i++) {
    if (motorstate[i].motor_type == 1) {
      current[i] = rm_move(i,&rm_send1,&rm_send2);
      //滑り検知//
      if (t_count > 5) {
        int16_t masatu;
        if (abs(motorstate[i].speed.rpm) > 0) {
          masatu = 110;
        }
        int32_t now_alpha = ( motorstate[i].speed.pre_rpm - motorstate[i].speed.rpm ) / (T_6 * t_count);
        float new_alpha = now_alpha * rpms_alpha + motorstate[i].speed.pre_alpha * (1 - rpms_alpha);
        int16_t alpha_meas = new_alpha;
        int16_t alpha_exp = (int16_t)((float)alpha_meas * C_value + (float)motorstate[i].speed.rpm * D_value + (float)masatu);
        motorstate[i].speed.pre_alpha = alpha_meas;
        if (i == 0) {
          view1 = alpha_meas / -10 + motorstate[i].speed.rpm / 10 + masatu;
          view2 = alpha_exp;
        } else if (i == 1) {
          //view2 = alpha_meas / -10;
        } else if (i == 2) {
          view3 = alpha_meas / -10;
        } else if (i == 3) {
          view4 = alpha_meas / -10;
          t_count = 0;
        }
        int8_t sign = (alpha_meas > 0) - (alpha_meas < 0);
        if ( sign * (alpha_exp - alpha_meas) > margin ) {
          //滑り
          view5 = sign * (alpha_exp - alpha_meas);
        }
        motorstate[i].speed.pre_rpm = motorstate[i].speed.rpm;
      }
      //滑り検知//
    } else if (motorstate[i].motor_type == 2 && (motorstate[i].mode == SPEED || motorstate[i].mode == ANGLE)) {
      rs_move(i);
    }
  }
  if (rm_send1) {
    tx_data[0] = (current[0] >> 8);
    tx_data[1] = (current[0] & 0xFF);
    tx_data[2] = (current[1] >> 8);
    tx_data[3] = (current[1] & 0xFF);
    tx_data[4] = (current[2] >> 8);
    tx_data[5] = (current[2] & 0xFF);
    tx_data[6] = (current[3] >> 8);
    tx_data[7] = (current[3] & 0xFF);
    can_send(0x200,tx_data,0,1);
  }
  if (rm_send2) {
    tx_data[0] = (current[4] >> 8);
    tx_data[1] = (current[4] & 0xFF);
    tx_data[2] = (current[5] >> 8);
    tx_data[3] = (current[5] & 0xFF);
    tx_data[4] = (current[6] >> 8);
    tx_data[5] = (current[6] & 0xFF);
    tx_data[6] = (current[7] >> 8);
    tx_data[7] = (current[7] & 0xFF);
    can_send(0x1FF,tx_data,0,1);
  }
}

void angle_pid_task(void) {
  for (int i = 0; i < 8; i++) {
    if (motorstate[i].mode == ANGLE) {
      motorstate[i].speed.target_rpm = pid(i,ANGLE);
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    t_count ++;
    //速度ループ
    countdown();
    speed_pid_task();
  } else if (htim->Instance == TIM16)
  {
    //角度ループ
    angle_pid_task();
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
  }
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
