
#include "ComplementaryFilter.h"
#include <cmath>

#define PI 										3.14159
#define RADIANS_TO_DEGREE(x) (x * 180.0 / PI)
#define ALPHA_COEF						0.88
void ComplementaryFilter_ComputeAngles( double*  AccData, double* GyroData, double SamplingFreq, double* OutputData)
{
	static double Gyro_Angle[3] = { 0.0, 0.0, 0.0 };
	double Acc_Angle[3] = { 0.0, 0.0, 0.0 };
	
	for ( uint8_t i = 0; i < 3; i++ )
	{
		Gyro_Angle[i] += GyroData[i] / SamplingFreq;
	}
	Acc_Angle[0] = RADIANS_TO_DEGREE(atan2(AccData[1], sqrt(pow(AccData[0],2) + pow(AccData[2],2))));
	Acc_Angle[1] = RADIANS_TO_DEGREE(atan2(AccData[0], sqrt(pow(AccData[1],2) + pow(AccData[2],2))));
	
	OutputData[0] = ALPHA_COEF * Gyro_Angle[0] + (1-ALPHA_COEF) * Acc_Angle[0];
	OutputData[1] = ALPHA_COEF * Gyro_Angle[1] + (1-ALPHA_COEF) * Acc_Angle[1];
	OutputData[2] = Gyro_Angle[2];

}