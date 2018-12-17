
#include "LSM303AGR.h"
#include "LSM303AGR_Driver.h"
#define SENSIVITY_2  4.0
#define RESOLUTION_10bit 1024.0

Sensor_t Sensor_Accel;


/**
 * @brief  This function initializes LSM303AGR_Accelerometer
 * @retval None
 */
void LSM303AGR_Accel_Init(void)
{
	// HP filter normal mode 
	SPI_Sensor_Write(Sensor_Accel,LSM303AGR_CTRL_REG2_A,0x00); 
	// interrupt disabled
	SPI_Sensor_Write(Sensor_Accel, LSM303AGR_CTRL_REG3_A,0x00); 
	// Block data update on, +-2g, normal mode, SPI3-wire enabled
	SPI_Sensor_Write(Sensor_Accel, LSM303AGR_CTRL_REG4_A,0x01); 
	// 100Hz data rate, X,Y,Z axix enabled
	SPI_Sensor_Write(Sensor_Accel, LSM303AGR_CTRL_REG1_A,0x57); 
	HAL_Delay(90);
}


/**
 * @brief  This function converts raw data to 'G' data
 * @param  Raw_Data: Pointer to array of raw data.
 * @param  Size: Size of an array.
 * @param  G_Data: Pointer to 'G' data.
 * @retval None
 */
void LSM303AGR_Data_Conv_To_G( int16_t * Raw_Data, uint8_t Size, double * G_Data )
{
	for(uint8_t i = 0; i < Size; i ++)
	{
		G_Data[i] = SENSIVITY_2 / RESOLUTION_10bit * (double)(Raw_Data[i]);
	}		
}


/**
 * @brief  This function reads data from all accelerometer axis.
 * @param  Raw_Data: Pointer to array of raw data.
 * @retval None
 */
void LSM303AGR_Accel_Read_X_Y_Z(int16_t *Raw_Data)
{
	uint8_t SPI_Data[6];
	SPI_Sensor_Read(Sensor_Accel, LSM303AGR_OUT_X_L_A, SPI_Data, 6);
	Raw_Data[0] = (int16_t)(SPI_Data[0] | (SPI_Data[1] << 8));
	Raw_Data[0] >>= 6;
	Raw_Data[1] = (int16_t)(SPI_Data[2] | (SPI_Data[3] << 8));
	Raw_Data[1] >>= 6;
	Raw_Data[2] = (int16_t)(SPI_Data[4] | (SPI_Data[5] << 8));
	Raw_Data[2] >>= 6;
}
