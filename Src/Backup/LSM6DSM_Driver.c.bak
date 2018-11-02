#include "LSM6DSM_Driver.h"
#include "LSM6DSM.h"

extern Sensor_t Sensor_LSM6DSM;

typedef struct sSens_coef 
{
	enum eM_Range M_Range;
	double Sensitivity;
} Sens_coef_t;

typedef struct sSensor_Configuration
{
	enum eSensor_Type Sensor_Type;
	enum eM_Range M_Range;
	
} Sensor_Configuration_t;

static Sens_coef_t Accel_Coef[] =
{
  {ACC_M_RANGE_2 ,	 0.061 },
	{ACC_M_RANGE_4 , 	 0.122 },
	{ACC_M_RANGE_8 , 	 0.244 },
	{ACC_M_RANGE_16 ,	 0.488 },
};

static Sens_coef_t Gyro_Coef[] =
{
	{GYRO_M_RANGE_125 ,		4.375},
	{GYRO_M_RANGE_250 ,		8.750},
	{GYRO_M_RANGE_500 ,		17.50},
	{GYRO_M_RANGE_1000 ,	35.00},
	{GYRO_M_RANGE_2000 ,	70.00},
};

static Sensor_Configuration_t Accel_Configuration;
static Sensor_Configuration_t Gyro_Configuration;

/**
 * @brief  This function initializes LSM6DSM accelerometer and gyrospcope
 * @retval None
 */
void LSM6DSM_Init(void)
{
	//SPI 3-Wire , auto_incr
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL3_C,0x0C);
	//I2C disabled, GYRO LPF1 enabled
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL4_C,0x06);  
	// high peformacnce mode, 104Hz, +-2g accel
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL1_XL,0x40); 
	// high performance 500dps 
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL2_G,0x44); 
	Accel_Configuration.M_Range = ACC_M_RANGE_2;
	Accel_Configuration.Sensor_Type = SENSOR_TYPE_ACC;
	Gyro_Configuration.Sensor_Type = SENSOR_TYPE_GYRO;
	Gyro_Configuration.M_Range = GYRO_M_RANGE_500;
}


/**
 * @brief  This function reads data from all accelerometer axis.
 * @param  Raw_Data: Pointer to array of raw data.
 * @retval None
 */
void LSM6DSM_Accel_Read_X_Y_Z(int16_t *Raw_Data)
{
	uint8_t SPI_Data[6];
	SPI_Sensor_Read(Sensor_LSM6DSM, LSM6DSM_OUTX_L_XL, SPI_Data, 6);
	Raw_Data[0] = (int16_t)(SPI_Data[0] | (SPI_Data[1] << 8));
	Raw_Data[1] = (int16_t)(SPI_Data[2] | (SPI_Data[3] << 8));
	Raw_Data[2] = (int16_t)(SPI_Data[4] | (SPI_Data[5] << 8));
}


/**
 * @brief  This function convert raw data to 'G' data.
 * @param  Raw_Data: Pointer to array of raw data.
 * @param  G_Data: Pointer to array of 'G' data.
 * @param  Accel_Range: Linear acceleration measurement range. Available values:
 * 				 ACC_M_RANGE_2, ACC_M_RANGE_4, ACC_M_RANGE_8, ACC_M_RANGE_16.
 * @retval None
 */
void LSM6DSM_Accel_Data_Conv_To_G( int16_t *Raw_Data, double *G_Data, enum eM_Range Accel_Range )
{
	for( int i = 0; i < 3; i++ )
	{
		G_Data[i] = Accel_Coef[Accel_Range].Sensitivity * (double)Raw_Data[i] / 1000.0;
	}
}


/**
 * @brief  This function reads data from all gyroscope axis.
 * @param  Raw_Data: Pointer to array of raw data.
 * @retval None
 */
void LSM6DSM_GYRO_Read_X_Y_Z(int16_t *Raw_Data)
{
	uint8_t SPI_Data[6];
	SPI_Sensor_Read(Sensor_LSM6DSM, LSM6DSM_OUTX_L_G, SPI_Data, 6);
	Raw_Data[0] = (int16_t)(SPI_Data[0] | (SPI_Data[1] << 8));
	Raw_Data[1] = (int16_t)(SPI_Data[2] | (SPI_Data[3] << 8));
	Raw_Data[2] = (int16_t)(SPI_Data[4] | (SPI_Data[5] << 8));
}


/**
 * @brief  This function convert raw data to 'DPS' data.
 * @param  Raw_Data: Pointer to array of raw data.
 * @param  DPS_Data: Pointer to array of 'DPS' data.
 * @param  Accel_Range: Angular rate measurement range. Available values:
 * 				 GYRO_M_RANGE_125, GYRO_M_RANGE_250, GYRO_M_RANGE_500, GYRO_M_RANGE_1000, GYRO_M_RANGE_2000.
 * @retval None
 */
void LSM6DSM_Gyro_Data_Conv_To_DPS( int16_t *Raw_Data, double *DPS_Data, enum eM_Range Gyro_Range )
{
	for( int i = 0; i < 3; i++ )
	{
		DPS_Data[i] = Accel_Coef[Gyro_Range-4].Sensitivity * (double)Raw_Data[i] / 1000.0;
	}
} 


/**
 * @brief  This function reads the LSM6DSM ID.
 * @param  Data: Pointer output value.
 * @retval None
 */
void LSM6DSM_Who_Am_I(uint8_t *Data)
{
	SPI_Sensor_Read(Sensor_LSM6DSM, LSM6DSM_WHO_AM_I_REG, Data, 1);
}


void LSM6DSM_Change_Configuration( enum eSensor_Type Sensor_Type, enum eM_Range M_Range)
{
	if ( Sensor_Type == SENSOR_TYPE_ACC )
	{
		Accel_Configuration.M_Range = M_Range;
	}
	else
	{
		Gyro_Configuration.M_Range = M_Range;
	}
}

enum eM_Range LSM6DSM_Get_Configuration( enum eSensor_Type Sensor_Type )
{
	if ( Sensor_Type == SENSOR_TYPE_ACC)
	{
		return Accel_Configuration.M_Range;
	}
	else
	{
		return Gyro_Configuration.M_Range;
	}
}

void LSM6DSM_Analyze_Register(uint8_t RegAdd, uint8_t RegVal)
{
	uint8_t temp;
	switch (RegAdd)
	{
		case LSM6DSM_CTRL1_XL:
				temp = (RegVal & 0x0B) >> 2;
				switch(temp)
				{
					case 0x00:
						Accel_Configuration.M_Range = ACC_M_RANGE_2;
						break;
					case 0x01:
						Accel_Configuration.M_Range = ACC_M_RANGE_16;
						break;
					case 0x02:
						Accel_Configuration.M_Range = ACC_M_RANGE_4;
						break;
					case 0x03:
						Accel_Configuration.M_Range = ACC_M_RANGE_8;
						break;
					default:
						break;
				}
			break;
			
		case LSM6DSM_CTRL2_G:
			temp = (RegVal & 0xB) >> 2;
			switch(temp)
			{
				case 0x00:
					Gyro_Configuration.M_Range = GYRO_M_RANGE_250;
					break;
				case 0x01:
					Gyro_Configuration.M_Range = GYRO_M_RANGE_500;
					break;
				case 0x02:
					Gyro_Configuration.M_Range = GYRO_M_RANGE_1000;
					break;
				case 0x03:
					Gyro_Configuration.M_Range = GYRO_M_RANGE_2000;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}