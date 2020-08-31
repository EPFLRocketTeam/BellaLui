/**
  ******************************************************************************
  * File Name          : FMPI2C.c
  * Description        : This file provides code for the configuration
  *                      of the FMPI2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fmpi2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FMPI2C_HandleTypeDef hfmpi2c1;
DMA_HandleTypeDef hdma_fmpi2c1_rx;


/* FMPI2C1 init function */
void MX_FMPI2C1_Init(void)
{

  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 500000;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_FMPI2C_MspInit(FMPI2C_HandleTypeDef* fmpi2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fmpi2cHandle->Instance==FMPI2C1)
  {
  /* USER CODE BEGIN FMPI2C1_MspInit 0 */

  /* USER CODE END FMPI2C1_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**FMPI2C1 GPIO Configuration
    PC6     ------> FMPI2C1_SCL
    PC7     ------> FMPI2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_FMPI2C1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_FMPI2C1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* FMPI2C1 clock enable */
    __HAL_RCC_FMPI2C1_CLK_ENABLE();


    /* FMPI2C1 DMA Init */
    /* FMPI2C1_RX Init */

    hdma_fmpi2c1_rx.Instance = DMA1_Stream2;
    hdma_fmpi2c1_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_fmpi2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_fmpi2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmpi2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmpi2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_fmpi2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_fmpi2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_fmpi2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_fmpi2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_fmpi2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    //__HAL_LINKDMA(fmpi2cHandle,hdmarx,hdma_fmpi2c1_rx);


    HAL_FMPI2CEx_EnableFastModePlus(FMPI2C_FASTMODEPLUS_SCL);
    HAL_FMPI2CEx_EnableFastModePlus(FMPI2C_FASTMODEPLUS_SDA);

  /* USER CODE BEGIN FMPI2C1_MspInit 1 */

  /* USER CODE END FMPI2C1_MspInit 1 */

  }
}

void HAL_FMPI2C_MspDeInit(FMPI2C_HandleTypeDef* fmpi2cHandle)
{

  if(fmpi2cHandle->Instance==FMPI2C1)
  {
  /* USER CODE BEGIN FMPI2C1_MspDeInit 0 */

  /* USER CODE END FMPI2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FMPI2C1_CLK_DISABLE();

    /**FMPI2C1 GPIO Configuration
    PC6     ------> FMPI2C1_SCL
    PC7     ------> FMPI2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);

    /* FMPI2C1 DMA DeInit */
    HAL_DMA_DeInit(fmpi2cHandle->hdmarx);
  /* USER CODE BEGIN FMPI2C1_MspDeInit 1 */

  /* USER CODE END FMPI2C1_MspDeInit 1 */
  }
}



/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
