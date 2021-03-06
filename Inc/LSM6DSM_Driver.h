#ifndef LSM6DSM_DRIVER_H_
#define LSM6DSM_DRIVER_H_

#include "spi.h"

enum eM_Range { ACC_M_RANGE_2, ACC_M_RANGE_4, ACC_M_RANGE_8, ACC_M_RANGE_16,
								GYRO_M_RANGE_125, GYRO_M_RANGE_250, GYRO_M_RANGE_500, GYRO_M_RANGE_1000, GYRO_M_RANGE_2000};
enum eSensor_Type { SENSOR_TYPE_ACC, SENSOR_TYPE_GYRO };



void LSM6DSM_Init(void);
void LSM6DSM_Accel_Read_X_Y_Z(int16_t *Raw_Data);
void LSM6DSM_GYRO_Read_X_Y_Z(int16_t *Raw_Data);
void LSM6DSM_Who_Am_I(uint8_t *Data);
void LSM6DSM_Gyro_Data_Conv_To_DPS( int16_t *Raw_Data, double *DPS_Data, enum eM_Range Gyro_Range );
void LSM6DSM_Accel_Data_Conv_To_G( int16_t *Raw_Data, double *G_Data, enum eM_Range Accel_Range );
enum eM_Range LSM6DSM_Get_Configuration( enum eSensor_Type Sensor_Type );
void LSM6DSM_Change_Configuration( enum eSensor_Type Sensor_Type, enum eM_Range M_Range);
void LSM6DSM_Analyze_Register(uint8_t RegAdd, uint8_t RegVal);
														
#endif
