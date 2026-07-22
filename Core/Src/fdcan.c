/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.c
 * @brief   This file provides code for the configuration
 *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "math.h"
#include <stdbool.h>
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
int8_t view_1;
int8_t view_2;
int8_t view_3;
int8_t view_4;
int8_t view_5;
int8_t view_6;
int16_t rpms[5] = {0};
int16_t rpms_sum;
int16_t now_rpm;
#define gear_ratio 19.204f
#define alpha 0.2f
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilter;

  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1 = 0x01F;
  sFilter.FilterID2 = 0xFE0;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);

  // フィルターに一致しないメッセージをつて破棄する設定
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);

  HAL_FDCAN_ActivateNotification(
      &hfdcan1,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
      0);

  HAL_FDCAN_Start(&hfdcan1);

  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 8;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 7;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 4;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 7;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 2;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  FDCAN_FilterTypeDef sFilter;

  // 重要度が高い通信
  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1 = 0xFFF;
  sFilter.FilterID2 = 0xFFF;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter);

  // 重要度が低い通信(モータードライバからのデータ)
  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 1;
  sFilter.FilterType = FDCAN_FILTER_RANGE;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilter.FilterID1 = 0x201;
  sFilter.FilterID2 = 0x208;

  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter);

  // フィルターに一致しないメッセージをすべて破棄する設定
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);

  HAL_FDCAN_ActivateNotification(
      &hfdcan2,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
      0);

  HAL_FDCAN_ActivateNotification(
      &hfdcan2,
      FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
      0);

  HAL_FDCAN_Start(&hfdcan2);

  /* USER CODE END FDCAN2_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = FDCAN1_Rx_Pin|FDCAN1_Tx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = FDCAN2_Rx_Pin|FDCAN2_Tx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, FDCAN1_Rx_Pin|FDCAN1_Tx_Pin);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, FDCAN2_Rx_Pin|FDCAN2_Tx_Pin);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[64];

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    HAL_FDCAN_GetRxMessage(
      hfdcan,
      FDCAN_RX_FIFO0,
      &RxHeader,
      RxData);

    if (hfdcan->Instance == FDCAN1)
    {
      uint32_t received_id = RxHeader.Identifier;
      int8_t mode = RxData[2];
      if (((received_id >> 4) & 0x01) == 1) {
        uint8_t gain_kind = (received_id & 0x0F);
        if (mode == ANGLE)
        {
          if (gain_kind == 0) {
            angle_gain.Kp = ((RxData[4] << 8) | RxData[5]);
          } else if (gain_kind == 1) {
            angle_gain.Ki = ((RxData[4] << 8) | RxData[5]);
          } else if (gain_kind == 2) {
            angle_gain.Kd = ((RxData[4] << 8) | RxData[5]);
          }
        }
        else if (mode == SPEED)
        {
          if (gain_kind == 0) {
            speed_gain.Kp = ((RxData[4] << 8) | RxData[5]);
          } else if (gain_kind == 1) {
            speed_gain.Ki = ((RxData[4] << 8) | RxData[5]);
          } else if (gain_kind == 2) {
            speed_gain.Kd = ((RxData[4] << 8) | RxData[5]);
          }
        }
      } else {
        if (mode == ANGLE)
        {
          double pre_target_angle = motorstate[received_id].target_angle;
          motorstate[received_id].mode = mode;
          motorstate[received_id].target_angle = (int16_t)((RxData[4] << 8) | RxData[5]);
          motorstate[received_id].half_target_angle = pre_target_angle - motorstate[received_id].target_angle;
        }
        else if (mode == SPEED)
        {
          return_rpms = true;
          motorstate[received_id].mode = mode;
          motorstate[received_id].target_rpm = (int16_t)((RxData[4] << 8) | RxData[5] ) * gear_ratio;
        }
      }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
      // --- FDCAN2 の処理 ---
    }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan2_, uint32_t RxFifo1ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[64];

  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
  {
    HAL_FDCAN_GetRxMessage(
        hfdcan2_,
        FDCAN_RX_FIFO1,
        &RxHeader,
        RxData);
    uint32_t received_id = RxHeader.Identifier & 0x00F;
    uint16_t angle_raw = (RxData[0] << 8) | RxData[1];
    now_rpm = (RxData[2] << 8) | RxData[3];
    //for(int i = 4; i > 0; i--) {
    //  if (motorstate[received_id - 1].sub_sit == Stop)
    //  {
    //    rpms[i] = (RxData[2] << 8) | RxData[3];
    //  } else {
    //    rpms[i] = rpms[i - 1];
    //  }
    //}
    //rpms[0] = (RxData[2] << 8) | RxData[3];
    //rpms_sum = 0;
    //for (int i = 0; i < 5; i++) {
    //  rpms_sum += rpms[i];
    //}
    //motorstate[received_id - 1].rpm = (float)rpms_sum / 5.0f;
    motorstate[received_id - 1].rpm = now_rpm * alpha + motorstate[received_id - 1].rpm * (1 - alpha);
    motorstate[received_id - 1].raw_angle = (float)angle_raw * 360.0f / 8192.0f;
    motorstate[received_id - 1].temp = (RxData[4] << 8) | RxData[5];
    float diff = motorstate[received_id - 1].raw_angle - motorstate[received_id - 1].pre_raw_angle;
    if (diff < -300)
    {
      motorstate[received_id - 1].resolution += 1;
    }
    else if (diff > 300)
    {
      motorstate[received_id - 1].resolution -= 1;
    }
    //    else {
    //      if (motorstate[received_id - 1].rpm > 0) {
    //        if (motorstate[received_id - 1].pre_raw_angle > 250 && motorstate[received_id - 1].raw_angle < 110) {
    //          motorstate[received_id - 1].r += 1;
    //        }
    //      } else if (motorstate[received_id - 1].rpm < 0) {
    //        if (motorstate[received_id - 1].pre_raw_angle < 110 && motorstate[received_id - 1].raw_angle > 250) {
    //          motorstate[received_id - 1].r -= 1;
    //        }
    //      }
    //    }
    motorstate[received_id - 1].angle = motorstate[received_id - 1].raw_angle + motorstate[received_id - 1].resolution * 360;
    motorstate[received_id - 1].pre_raw_angle = motorstate[received_id - 1].raw_angle;
    if (motorstate[received_id - 1].sub_sit == Stop)
    {
      motorstate[received_id - 1].sub_sit = Move;
      motorstate[received_id - 1].angle_zero = motorstate[received_id - 1].angle;
    }
    motorstate[received_id - 1].angle = fmod(motorstate[received_id - 1].angle - motorstate[received_id - 1].angle_zero, 360 * gear_ratio);
    if (motorstate[received_id - 1].angle < 0)
    {
      motorstate[received_id - 1].angle += 360 * gear_ratio;
    }
  }
}
/* USER CODE END 1 */

