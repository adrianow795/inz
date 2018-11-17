#include <math.h>
#include "Quaternion.h"

static volatile double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
static volatile float q0f = 1.0, q1f = 0.0, q2f = 0.0, q3f = 0.0;
static volatile float SamplePerDiv4 = (1.0f/8000.0f)/4.0f;

void QuaternionMultiple(double qin0, double qin1, double qin2, double qin3) {
	double x, y, z, w;
	
	x =	qin1 * q0 + qin2 * q3 - qin3 * q2 + qin0 * q1;
	y = -qin1 * q3 + qin2 * q0 + qin3 * q1 + qin0 * q2;
	z =	qin1 * q2 - qin2 * q1 + qin3 * q0 + qin0 * q3;
	w = -qin1 * q1 - qin2 * q2 - qin3 * q3 + qin0 * q0;
	
	q0 = w;
	q1 = x;
	q2 = y;
	q3 = z;
}

void QuaternionMultiplef(float qin0, float qin1, float qin2, float qin3) {
	float x, y, z, w;
	
	x =	qin1 * q0f + qin2 * q3f - qin3 * q2f + qin0 * q1f;
	y = -qin1 * q3f + qin2 * q0f + qin3 * q1f + qin0 * q2f;
	z =	qin1 * q2f - qin2 * q1f + qin3 * q0f + qin0 * q3f;
	w = -qin1 * q1f - qin2 * q2f - qin3 * q3f + qin0 * q0f;
	
	q0f = w;
	q1f = x;
	q2f = y;
	q3f = z;
}

void QuaternionUpdate(float deltax, float deltay, float deltaz)
{
	static float deltax_old = 0, deltay_old = 0, deltaz_old = 0;
	double cx, sx , cy, sy, cz, sz;
	double q0rot, q1rot, q2rot, q3rot;
	double recipNorm;
	
	//sample period divided by 4
	cx = cos((double)(deltax+deltax_old)*SamplePerDiv4);
	sx = sin((double)(deltax+deltax_old)*SamplePerDiv4);
	cy = cos((double)(deltay+deltay_old)*SamplePerDiv4);
	sy = sin((double)(deltay+deltay_old)*SamplePerDiv4);
	cz = cos((double)(deltaz+deltaz_old)*SamplePerDiv4);
	sz = sin((double)(deltaz+deltaz_old)*SamplePerDiv4);
	
	deltax_old = deltax;
	deltay_old = deltay;
	deltaz_old = deltaz;
	
	q0rot = cz * cy * cx + sz * sy * sx;
	q1rot = cz * sy * cx - sz * cy * sx;
	q2rot = cz * cy * sx + sz * sy * cx;
	q3rot = sz * cy * cx - cz * sy * sx;
	
	QuaternionMultiple(q0rot, q1rot, q2rot, q3rot);
	
	recipNorm = 1.0f/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;	
}

void QuaternionUpdatef(float deltax, float deltay, float deltaz)
{
	static float deltax_old = 0, deltay_old = 0, deltaz_old = 0;
	float dx, dy, dz;
	float cx, sx , cy, sy, cz, sz;
	float q0rot, q1rot, q2rot, q3rot;
	float recipNorm;
	
	dx = deltax+deltax_old;
	dy = deltay+deltay_old;
	dz = deltaz+deltaz_old;

	deltax_old = deltax;
	deltay_old = deltay;
	deltaz_old = deltaz;
	
	//sample period divided by 4
	cx = cosf(dx*SamplePerDiv4);
	sx = sinf(dx*SamplePerDiv4);
	cy = cosf(dy*SamplePerDiv4);
	sy = sinf(dy*SamplePerDiv4);
	cz = cosf(dz*SamplePerDiv4);
	sz = sinf(dz*SamplePerDiv4);	

	q0rot = cz * cy * cx + sz * sy * sx;
	q1rot = cz * sy * cx - sz * cy * sx;
	q2rot = cz * cy * sx + sz * sy * cx;
	q3rot = sz * cy * cx - cz * sy * sx;
	
	QuaternionMultiplef(q0rot, q1rot, q2rot, q3rot);
	
	recipNorm = 1.0f/sqrtf(q0f * q0f + q1f * q1f + q2f * q2f + q3f * q3f);
	q0f *= recipNorm;
	q1f *= recipNorm;
	q2f *= recipNorm;
	q3f *= recipNorm;	
}

// Access functions
void QuaternionGetQ(float* qin0, float* qin1, float* qin2, float* qin3)
{
		*qin0 = (float)q0;
		*qin1 = (float)q1;
		*qin2 = (float)q2;
		*qin3 = (float)q3;
}

void QuaternionGetQf(float* qin0, float* qin1, float* qin2, float* qin3)
{
		*qin0 = q0f;
		*qin1 = q1f;
		*qin2 = q2f;
		*qin3 = q3f;
}

void QuaternionReset(void)
{
		q0 = 1.0;
		q1 = 0.0;
		q2 = 0.0;
		q3 = 0.0;
		q0f = 1.0;
		q1f = 0.0;
		q2f = 0.0;
		q3f = 0.0;
}

void QuaternionSetSamplePer(float period) 
{
	SamplePerDiv4 = period/4.0f;
}

float QuaternionGetSamplePer(void) 
{
	return (SamplePerDiv4 * 4.0f);
}
