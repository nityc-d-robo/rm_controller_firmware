/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOC
#define FDCAN2_Rx_Pin GPIO_PIN_12
#define FDCAN2_Rx_GPIO_Port GPIOB
#define FDCAN2_Tx_Pin GPIO_PIN_13
#define FDCAN2_Tx_GPIO_Port GPIOB
#define FDCAN1_Rx_Pin GPIO_PIN_11
#define FDCAN1_Rx_GPIO_Port GPIOA
#define FDCAN1_Tx_Pin GPIO_PIN_12
#define FDCAN1_Tx_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define GEAR_RATIO 19.204f
#define ENCODER_RESOLUTION 8192.0f
#define alpha 0.02f //0.2fから変更
#define rpms_alpha 0.2f
#define T_6 0.004f //t_6周期
#define T_16 0.008f //t_16周期
#define MAX_IE 1500
#define MIN_IE (-MAX_IE)
#define MAX_POWER 10000
#define MIN_POWER (-MAX_POWER)
#define RM_MAX_RPM 3000
#define RM_MIN_RPM (-RM_MAX_RPM)
#define RS_MAX_RPM 450
#define RS_MIN_RPM (-RS_MAX_RPM)
#define RESOLUTION 6913.0f
#define MAX_RAD_S 50
#define MIN_RAD_S (-MAX_RAD_S)
#define C_value -0.1f
#define D_value 0.1f
#define margin 300

typedef enum
{
  NONE,
  PWM,
  CURRENT,
  SPEED,
  ANGLE,
} Mode;

typedef struct
{
  float Kp;
  float Ki;
  float Kd;
} Gain;

typedef struct 
{
  bool rx_flag;
  uint32_t rx_id;
  uint8_t rx_data[8];
} RxData;

typedef struct{
  RxData main_rx[8]; //メインからの命令
  RxData rm_rx[8]; //ロボマスからのフィードバック
  RxData rs_rx[8]; //ロボストライドからのフィードバック
} RxDatas;

typedef struct{
  float ie;
  float e_pre;
} PIDState;

typedef struct{
  int16_t rpm;
  int16_t pre_rpm; //追加
  int16_t target_rpm;
  int16_t pre_alpha;
  Gain speed_gain;
  PIDState speed_pid_state;
} Speed;

typedef struct{
  float angle;
  float angle_zero;
  float target_angle;
  float pre_angle_360;
  int16_t resolution;
  Gain angle_gain;
  PIDState angle_pid_state;
} Angle;

typedef struct{
  int16_t current;
  int16_t target_current;
} Current;

typedef struct{
  int8_t motor_type;
  Mode mode;
  Angle angle;
  Speed speed;
  Current current;
  bool first_move;
  uint16_t motor_can_move;
} MotorState;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
