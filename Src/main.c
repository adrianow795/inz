/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32l4xx_hal.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "LSM303AGR_Driver.h"
#include <stdio.h>
#include "LSM6DSM_Driver.h"
#include "String_Decoder.h"
#include <cmath>
//#include "SensorTile.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t ReceivedData[60]; // Tablica przechowujaca odebrane dane
uint8_t ReceivedDataFlag = 0; // Flaga informujaca o odebraniu danych
int16_t Raw_Data_Accel[3];
int16_t Raw_Data_Gyro[3];
double G_Data_Accel[3];
double DPS_Data_Gyro[3];
uint8_t SPI_Data[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t FindNull ( char * Buf);
void CleanBuffer(char* Buf, uint8_t Size);
void WriteIntoBuffer(char* Buf, char* str, uint8_t Buf_Size);
void Request_handle(void);
uint8_t FindNewLine( char* str);
void LSM6DSM_Callibrate(int16_t *AccOffset, int16_t *GyroOffset);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//extern UART_HandleTypeDef huart5;
	Sensor_t Sensor_Accel;
  Sensor_t Sensor_LSM6DSM;
	extern Request_t Request;
	volatile uint8_t LSM6DSM_DataReady = 1;
	uint8_t StatusReg = 0; 
	uint8_t* StatusRegPointer;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	Sensor_Accel.GPIOx = CS_A_GPIO_Port;
	Sensor_Accel.GPIO_Pin = CS_A_Pin;
	Sensor_Accel.Sensor = SENSOR_ACCEL;
	
	Sensor_LSM6DSM.GPIOx = CS_AG_GPIO_Port;
	Sensor_LSM6DSM.GPIO_Pin = CS_AG_Pin;
	Sensor_LSM6DSM.Sensor = SENSOR_AG;
	
	char* Tokens[MAX_ARGS_NUMBER+1];
	char Buf[60] ;
	uint8_t Value[2];
	int16_t AccOffset[3] = {0,0,0};
	int16_t GyroOffset[3] = {0,0,0};
	StatusRegPointer = &StatusReg;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	SPI_Initialization();
	HAL_Delay(15);
	//LSM303AGR_Accel_Init();
	LSM6DSM_Init();
	LSM6DSM_Who_Am_I(Value);
	sprintf(Buf,"Who_I_AM = %x \r\n",Value[0]);
	CDC_Transmit_FS((uint8_t *)Buf,60);
	HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	
	char TxBufA[100], TxBufG[50];
	LSM6DSM_Callibrate(AccOffset,GyroOffset);
	
  while (1)
  {
		/*
		LSM303AGR_Accel_Read_X_Y_Z(Raw_Data_Accel);
		LSM303AGR_Data_Conv_To_G(Raw_Data_Accel,3,G_Data);
		sprintf(Buf,"X = %0.2f, Y = %0.2f, Z = %0.2f ;\n\r",G_Data[0],G_Data[1],G_Data[2]);
		CDC_Transmit_FS((uint8_t *)Buf,60);
		
		LSM6DSM_Accel_Read_X_Y_Z(Raw_Data_Accel);
		LSM6DSM_Accel_Data_Conv_To_G(Raw_Data_Accel, G_Data, ACC_M_RANGE_2);
		sprintf(Buf,"X = %0.2f, Y = %0.2f, Z = %0.2f ;\n\r",G_Data[0],G_Data[1],G_Data[2]);
		CDC_Transmit_FS((uint8_t *)Buf,60);
		for(int i = 0 ; i < 60; i++)
		{
			Buf[i] = 0;
		}
		
		LSM6DSM_GYRO_Read_X_Y_Z(Raw_Data_Gyro);
		LSM6DSM_Gyro_Data_Conv_To_DPS(Raw_Data_Gyro, DPS_Data, GYRO_M_RANGE_500);
		sprintf(Buf,"Pitch = %3.2f, Roll = %3.2f, Yaw = %3.2f ;\n\r",DPS_Data[0],DPS_Data[1],DPS_Data[2]);

		CDC_Transmit_FS((uint8_t *)Buf,60);
		for(int i = 0 ; i < 60; i++)
		{
			Buf[i] = 0;
		}
		*/
		/*
		if(ReceivedDataFlag == 1)
		{
			ReceivedDataFlag = 0;
			Find_Tokens((char*)ReceivedData,Tokens);
			if(OK_STATUS == Decode_Tokens(Tokens))
			{
				Request_handle();
			}
			else
			{
				CDC_Transmit_FS((uint8_t*)"Unknown command\r\n",strlen("Unknown command\r\n")+1);
			}
			
			*/
			
			/*sprintf(Buf,"Komenda: %d,\n\rLiczba argumentow: %d\r\n",Request.Command, Request.Number_Of_Arguments);
			CDC_Transmit_FS((uint8_t*)Buf,FindNull(Buf));
			
			for (int i = 0; i < Request.Number_Of_Arguments; i++)
			{
				CleanBuffer(Buf,60);
				sprintf(Buf,"Arg%d: 0x%x\r\n",i+1,Request.Arguments[i]);
				CDC_Transmit_FS((uint8_t*)Buf,FindNull(Buf));
				HAL_Delay(10);
				HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12);
			}			
		} */
		
		
    //HAL_Delay(100);
//		SPI_Sensor_Read(Sensor_Accel, 0x1E,StatusRegPointer,1);
    if(LSM6DSM_DataReady)
		{
		  LSM6DSM_DataReady = 0;
			LSM6DSM_Accel_Read_X_Y_Z(Raw_Data_Accel);
			LSM6DSM_Accel_Data_Conv_To_G(Raw_Data_Accel,G_Data_Accel,LSM6DSM_Get_Configuration(SENSOR_TYPE_ACC));
			LSM6DSM_GYRO_Read_X_Y_Z(Raw_Data_Gyro);
			for(uint8_t i = 0; i < 3; i++)
			{
				Raw_Data_Gyro[i] -= GyroOffset[i];
			}
			LSM6DSM_Gyro_Data_Conv_To_DPS(Raw_Data_Gyro,DPS_Data_Gyro,LSM6DSM_Get_Configuration(SENSOR_TYPE_GYRO));
			sprintf(TxBufA,"A:%1.3f;%1.3f;%1.3f;G:%3.1f;%3.1f;%3.1f\n",G_Data_Accel[0],G_Data_Accel[1],G_Data_Accel[2],DPS_Data_Gyro[0],DPS_Data_Gyro[1],DPS_Data_Gyro[2]);
		  CDC_Transmit_FS((uint8_t*)TxBufA,FindNewLine(TxBufA));
		}
		
		
		//sprintf(TxBufA,"G:%3.1f;%3.1f;%3.1f\r\n",DPS_Data_Gyro[0],DPS_Data_Gyro[1],DPS_Data_Gyro[2]);
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void Request_handle(void)
{
	char Response[60];
	char Aux_Buf[5];
	
	switch (Request.Command)
	{
		// write  Reg_Add  reg_number  values
		case CMD_WRITE:
			if(Request.Arguments[1] > 1)
			{
				for ( uint8_t i = 0; i < Request.Arguments[1]; i++)
				{
					SPI_Sensor_Write(Sensor_LSM6DSM, Request.Arguments[0]+i, Request.Arguments[2+i]);
					LSM6DSM_Analyze_Register(Request.Arguments[0]+i, Request.Arguments[2+i]);
				}
			}
			else
			{
				SPI_Sensor_Write(Sensor_LSM6DSM, Request.Arguments[0], Request.Arguments[2]);
			}
			CleanBuffer(Response,60);
			sprintf(Response,"ok\r\n");
			CDC_Transmit_FS((uint8_t*)Response,FindNull(Response));
			break;
		
		// read  Reg_Add   Regs_number(max 10)
		case CMD_READ:
			SPI_Sensor_Read(Sensor_LSM6DSM, Request.Arguments[0], SPI_Data, Request.Arguments[1]);
			CleanBuffer(Response,60);
			sprintf(Response,"0x%x ",SPI_Data[0]);
			if(Request.Arguments[1] > 1)
			{
				for ( int i = 1; i < Request.Arguments[1]; i++)
				{
					sprintf(Aux_Buf,"0x%x ",SPI_Data[i]);
					strcat(Response, Aux_Buf);
				}
			}
			strcat(Response, "\r\n");
			CDC_Transmit_FS((uint8_t*)Response,FindNull(Response));
			break;
		
		// accel_m_xyz
		case CMD_ACCEL_M_XYZ:
			LSM6DSM_Accel_Read_X_Y_Z(Raw_Data_Accel);
			CleanBuffer(Response,60);
			sprintf(Response,"0x%x 0x%x 0x%x\n\r",Raw_Data_Accel[0],Raw_Data_Accel[1],Raw_Data_Accel[2]);
			CDC_Transmit_FS((uint8_t*)Response,FindNull(Response));
			break;
		
		// accel_m_xyz_g
		case CMD_ACCEL_M_XYZ_G:
			LSM6DSM_Accel_Read_X_Y_Z(Raw_Data_Accel);
			LSM6DSM_Accel_Data_Conv_To_G(Raw_Data_Accel,G_Data_Accel, LSM6DSM_Get_Configuration(SENSOR_TYPE_ACC));
			sprintf(Response,"%2.3f %2.3f %2.3f\n\r",G_Data_Accel[0],G_Data_Accel[1],G_Data_Accel[2]);
			CDC_Transmit_FS((uint8_t*)Response,FindNull(Response));
			break;
		
		// gyro_m_pyr
		case CMD_GYRO_M_PYR_DPS:
			LSM6DSM_GYRO_Read_X_Y_Z(Raw_Data_Gyro);
			LSM6DSM_Gyro_Data_Conv_To_DPS(Raw_Data_Gyro, DPS_Data_Gyro, LSM6DSM_Get_Configuration(SENSOR_TYPE_GYRO));
			sprintf(Response,"%4.3f %4.3f %4.3f\n\r",DPS_Data_Gyro[0],DPS_Data_Gyro[1],DPS_Data_Gyro[2]);
			CDC_Transmit_FS((uint8_t*)Response,FindNull(Response));
			break;
		
		case CMD_GYRO_M_PYR:
			LSM6DSM_GYRO_Read_X_Y_Z(Raw_Data_Gyro);
			CleanBuffer(Response,60);
			sprintf(Response,"0x%x 0x%x 0x%x\n\r",Raw_Data_Gyro[0],Raw_Data_Gyro[1],Raw_Data_Gyro[2]);
			CDC_Transmit_FS((uint8_t*)Response,FindNull(Response));
		default:
			break;
	}
	
}

void WriteIntoBuffer(char* Buf, char* str, uint8_t Buf_Size)
{
	uint8_t i = 0;
	while ( str[i] != NULL && i < Buf_Size)
	{
		Buf[i] = str[i];
	}
}

void CleanBuffer(char* Buf, uint8_t Size)
{
	for (int i = 0; i < Size; i++)
	{
		Buf[i] = 0;
	}
}

uint8_t FindNull ( char * Buf)
{
	int i = 0; 
	while (Buf[i] != 0)
	{
		i++;
	}
	return i;
}

uint8_t FindNewLine( char* str)
{
	int i = 0;
	while( str[i] != '\n')
	{
		i++;
	}
	return ++i;
}

void LSM6DSM_Callibrate(int16_t *AccOffset, int16_t *GyroOffset)
{
	uint8_t SamplesCounter = 0;
	int32_t AccSum[3] = {0,0,0};
	int32_t GyroSum[3] = {0,0,0};
	while(SamplesCounter < 50)
	{
		if(LSM6DSM_DataReady)
		{
			LSM6DSM_DataReady = 0;
			LSM6DSM_Accel_Read_X_Y_Z(Raw_Data_Accel);
			LSM6DSM_GYRO_Read_X_Y_Z(Raw_Data_Gyro);
			SamplesCounter++;
		}
	}
	SamplesCounter = 0;
	
	while(SamplesCounter < 50)
	{
		if(LSM6DSM_DataReady)
		{
			LSM6DSM_DataReady = 0;
			LSM6DSM_Accel_Read_X_Y_Z(Raw_Data_Accel);
			LSM6DSM_GYRO_Read_X_Y_Z(Raw_Data_Gyro);
			SamplesCounter++;
			for( uint8_t i = 0; i < 3; i++)
			{
				AccSum[i] += (int32_t)Raw_Data_Accel[i];
				GyroSum[i] += (int32_t)Raw_Data_Gyro[i];
			}
		}
	}
	for ( uint8_t i = 0; i < 3; i++)
	{
		AccOffset[i] = (int16_t)(AccSum[i] / 50); 
		GyroOffset[i] = (int16_t)(GyroSum[i] / 50);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
