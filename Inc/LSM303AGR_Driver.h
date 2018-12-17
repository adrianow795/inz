#ifndef LSM303AGR_ACCEL_DRIVER_H_
#define LSM303AGR_ACCEL_DRIVER_H_

#include "spi.h"
extern Sensor_t Sensor_Accel;
void LSM303AGR_Accel_Init(void);
void LSM303AGR_Accel_Read_X_Y_Z(int16_t *Raw_Data);
void LSM303AGR_Data_Conv_To_G( int16_t * Raw_Data, uint8_t Size, double * G_Data );

#endif

