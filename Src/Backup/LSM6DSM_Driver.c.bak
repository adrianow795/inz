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
	/*
-Accel - 833Hz, -+8g   
	CTRL1_XL : 0x7C
-Gyro - 833Hz, -+2000dps
	CTRL2_G : 0x7C
-Common - continuous update, int high, push-pull int2, 3-wire SPI, 
		  automatically incremented address during reading, LSB Endian
	CTRL3_C : 0x0C
-Common - SPI only, en LPF1 for Gyro
	CTRL4_C: 0x06
-Common - no rounding, self-tests disabled	
	CTRL5_C :0x00
-Common - gyro LPF1 bandwidth 245Hz
	CTRL6_C : 0x0
-Gyro - high-perf mode en,  HPF en, 1.04Hz -cutoff freq
	CTRL7_G : 0x60
-Accel - LPF2 en, ODR/9
	CTRL8_XL: 0xC0
-Interrupt - gyro, accel
	INT2_CTRL: 0x03
	*/

	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL3_C,0x0C);
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL1_XL,0x7C);//lpf1_sel
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL2_G,0x7C);
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL4_C,0x06); //
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL6_C,0x03);
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL7_G,0x70);
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL8_XL,0xC0);
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_INT2_CTRL,0x03);
	/*
	//I2C disabled, GYRO LPF1 enabled
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL4_C,0x03);  //without filters -> 0x02
	//LPF gyro bandwitdht
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL6_G,0x03); 
	// high peformacnce mode, 104Hz, +-2g accel --> 0x40
	// high perf. 833Hz, +- 4g accel --> 0x78
	// 416 --> 0x68
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL1_XL,0x68); 
	// high performance 500dps ,104hz 0x44
	// high perf, 1000dps, 833Hz 0x78
	//										 416Hz 0x68
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL2_G,0x68); 
	// interrupts data ready - acc, gyro 
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_INT2_CTRL,0x03); 
	
	// HP filtrer gyro
	SPI_Sensor_Write(Sensor_LSM6DSM,LSM6DSM_CTRL7_G,0x00);  // without filters -> 0x00
	//LPF for gyro - cut off freq
	*/
	
	Accel_Configuration.M_Range = ACC_M_RANGE_8;
	Accel_Configuration.Sensor_Type = SENSOR_TYPE_ACC;
	Gyro_Configuration.Sensor_Type = SENSOR_TYPE_GYRO;
	Gyro_Configuration.M_Range = GYRO_M_RANGE_2000;
	
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
		
		DPS_Data[i] = Gyro_Coef[Gyro_Range-4].Sensitivity * (double)Raw_Data[i] / 1000.0;
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