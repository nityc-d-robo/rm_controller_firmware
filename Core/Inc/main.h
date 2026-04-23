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

  typedef enum
  {
    INIT,
    STATUS,
    PWM,
    SPEED,
    ANGLE,
    LIM_SW,
  } mode;

  typedef struct
  {
    double  ie;
    double e_pre;
  } PIDState;

  typedef struct
  {
    mode mode;
    double angle;
    double r;
    double pre_angle;
    double raw_angle;
    int16_t rpm;
    int8_t temp;
    double target_angle;
    int16_t target_rpm;
    PIDState angle_pid_state;
    PIDState speed_pid_state
  } MotorState;

  typedef struct {
    float Kp;
    float Ki;
    float Kd;
  } Gain;

  typedef enum {
    Stop,
    Move,
  } Sit;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
