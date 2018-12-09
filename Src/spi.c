/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "main.h"

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB15     ------> SPI2_MOSI
    PB13     ------> SPI2_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB15     ------> SPI2_MOSI
    PB13     ------> SPI2_SCK 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15|GPIO_PIN_13);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void SPI_Initialization(void)
{
	 SPI_1LINE_TX(&hspi2);
	__HAL_SPI_ENABLE(&hspi2);
}

/**
 * @brief  This function enables slave
 * @param  GPIOx: Slave Port.
 * @param  GPIO_Pin: Slave Pin.
 * @retval None
 */
void SPI_Enable_Slave(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}


/**
 * @brief  This function disables slave
 * @param  GPIOx: Slave Port.
 * @param  GPIO_Pin: Slave Pin.
 * @retval None
 */
void SPI_Disable_Slave(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}


/**
 * @brief  This function writes a single byte on SPI 3-wire.
 * @param  Data: Data to send
 * @retval None
 */
void SPI_Write_Byte(uint8_t Data)
{
	while ((hspi2.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE );
	*((__IO uint8_t*) &hspi2.Instance->DR) = Data;
	while ((hspi2.Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
	while((hspi2.Instance->SR & SPI_FLAG_BSY ) == SPI_FLAG_BSY);
}


/**
 * @brief  This function reads a single byte from SPI 3-wire.
 * @param  Data: Data to send
 * @retval None
 */
void SPI_Read_Byte(uint8_t *Data)
{
	__disable_irq();
  __HAL_SPI_ENABLE(&hspi2);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(&hspi2);
  __enable_irq();
	
	while ((hspi2.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) {};
  /* read the received data */
  *Data = *(__IO uint8_t *) &hspi2.Instance->DR;
  while ((hspi2.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}


/**
 * @brief  This function reads multiple bytes from SPI 3-wire.
 * @param  *Data: Pointer to data to send
 * @param  BytesNumber: Number of bytes to read
 * @retval None
 */
void SPI_Read_n_Bytes(uint8_t *Data, uint8_t BytesNumber)
{
	__disable_irq();
  __HAL_SPI_ENABLE(&hspi2);
	while (BytesNumber > 1U)
  {
    if (hspi2.Instance->SR & SPI_FLAG_RXNE)
    {
      *Data = *(__IO uint8_t *) &hspi2.Instance->DR;
      Data += sizeof(uint8_t);
      BytesNumber--;
    }
  }
	
	__DSB();
  __DSB();
  __HAL_SPI_DISABLE(&hspi2);
  __enable_irq();
	
	while ((hspi2.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE){};
  *Data = *(__IO uint8_t *) &hspi2.Instance->DR;
  while ((hspi2.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY){};
}


/**
 * @brief  This function writes data to accelerometer register via SPI 3-wire.
 * @param  RegAdd: Address of the register
 * @param  Data: Data to write
 * @retval None
 */
void SPI_Accel_Write(uint8_t RegAdd, uint8_t Data)
{
	SPI_Enable_Slave(CS_A_GPIO_Port,CS_A_Pin);
	SPI_Write_Byte(RegAdd);
	SPI_Write_Byte(Data);
	SPI_Disable_Slave(CS_A_GPIO_Port,CS_A_Pin);
}



/**
 * @brief  This function writes data to sensor register via SPI 3-wire.
 * @param  Sensor: Sensor type and pinout.
 * @param  RegAdd: Address of the register
 * @param  Data: Data to write
 * @retval None
 */
void SPI_Sensor_Write(Sensor_t Sensor, uint8_t RegAdd, uint8_t Data)
{
	SPI_Enable_Slave(Sensor.GPIOx, Sensor.GPIO_Pin);
	SPI_Write_Byte(RegAdd);
	SPI_Write_Byte(Data);
	SPI_Disable_Slave(Sensor.GPIOx, Sensor.GPIO_Pin);
}


/**
 * @brief  This function reads data from accelerometer via SPI 3-wire.
 * @param  RegAdd: Address of the register
 * @param  *Data: Pointer to data to write
 * @param  BytesNumber: Number of bytes to read
 * @retval None
 */
void SPI_Accel_Read(uint8_t RegAdd, uint8_t *Data, uint8_t BytesNumber)
{
	SPI_Enable_Slave(CS_A_GPIO_Port,CS_A_Pin);
	if (BytesNumber > 1)
	{
		SPI_Write_Byte(0xC0 | RegAdd);
	}
	else
	{
		SPI_Write_Byte(0x80 | RegAdd);
	}
	
	
	
	__HAL_SPI_DISABLE(&hspi2);
  SPI_1LINE_RX(&hspi2);
	if(BytesNumber > 1U)
  {
    SPI_Read_n_Bytes(Data, BytesNumber);
  }
  else
  {
    SPI_Read_Byte(Data);
  }
	SPI_Disable_Slave(CS_A_GPIO_Port,CS_A_Pin);
	
	SPI_1LINE_TX(&hspi2);
  __HAL_SPI_ENABLE(&hspi2);
	
}
/**
 * @brief  This function reads data from accelerometer via SPI 3-wire.
 * @param  Sensor: Sensor type and pinout.
 * @param  RegAdd: Address of the register
 * @param  *Data: Pointer to data to write
 * @param  BytesNumber: Number of bytes to read
 * @retval None
 */

void SPI_Sensor_Read(Sensor_t Sensor, uint8_t RegAdd, uint8_t *Data, uint8_t BytesNumber)
{
	SPI_Enable_Slave(Sensor.GPIOx, Sensor.GPIO_Pin);
	if (BytesNumber > 1 && Sensor.Sensor != SENSOR_AG)
	{
		SPI_Write_Byte(0xC0 | RegAdd);  
	}
	else
	{
		SPI_Write_Byte(0x80 | RegAdd);
	}
	
	
	
	__HAL_SPI_DISABLE(&hspi2);
  SPI_1LINE_RX(&hspi2);
	if(BytesNumber > 1U)
  {
    SPI_Read_n_Bytes(Data, BytesNumber);
  }
  else
  {
    SPI_Read_Byte(Data);
  }
	SPI_Disable_Slave(Sensor.GPIOx, Sensor.GPIO_Pin);
	
	SPI_1LINE_TX(&hspi2);
  __HAL_SPI_ENABLE(&hspi2);
	
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
