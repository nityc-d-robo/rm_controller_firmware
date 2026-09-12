/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.h
 * @brief   This file contains all the function prototypes for
 *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
  /* USER CODE END Includes */

  extern FDCAN_HandleTypeDef hfdcan1;

  extern FDCAN_HandleTypeDef hfdcan2;

  /* USER CODE BEGIN Private defines */
  extern MotorState motorstate[8];
  extern Gain speed_gain;
  extern Gain angle_gain;
  extern bool return_rpms;
  extern volatile uint8_t main1_rx_data[8];
  extern volatile bool main1_rx_flag;
  extern volatile uint32_t main1_rx_id;
  extern volatile uint8_t main2_rx_data[8];
  extern volatile bool main2_rx_flag;
  extern volatile uint32_t main2_rx_id;
  extern volatile uint8_t motor1_rx_data[8];
  extern volatile bool motor1_rx_flag;
  extern volatile uint32_t motor1_rx_id;
  extern volatile uint8_t motor2_rx_data[8];
  extern volatile bool motor2_rx_flag;
  extern volatile uint32_t motor2_rx_id;
  /* USER CODE END Private defines */

  void MX_FDCAN1_Init(void);
  void MX_FDCAN2_Init(void);

  /* USER CODE BEGIN Prototypes */

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
