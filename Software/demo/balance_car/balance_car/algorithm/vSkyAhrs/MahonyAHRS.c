#include "MahonyAHRS.h"
#include <math.h>

#define sampleFreq 512.0f	   // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

volatile float twoKp = twoKpDef;										   // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;										   // 2 * integral gain (Ki)
volatile float mq0 = 1.0f, mq1 = 0.0f, mq2 = 0.0f, mq3 = 0.0f;				   // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

float invSqrtM(float x);

void MahonyAHRSClear()
{
	mq0 = 1.0f;
	mq1 = 0.0f;
	mq2 = 0.0f;
	mq3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
}
float MahonyAHRSupdate(float *accel, float *gyro, float *mag, float *angle, float Cycle)
{
	float coefficient;
	float recipNorm;
	float mq0mq0, mq0mq1, mq0mq2, mq0mq3, mq1mq1, mq1mq2, mq1mq3, mq2mq2, mq2mq3, mq3mq3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float gx,gy,gz,ax,ay,az,mx,my,mz;
	ax = accel[0];
	ay = accel[1];
	az = accel[2];

	gx = gyro[0];
	gy = gyro[1];
	gz = gyro[2];

	mx = mag[0];
	mx = mag[1];
	mx = mag[2];

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		return MahonyAHRSupdateIMU(accel, gyro, angle, Cycle);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrtM(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrtM(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		mq0mq0 = mq0 * mq0;
		mq0mq1 = mq0 * mq1;
		mq0mq2 = mq0 * mq2;
		mq0mq3 = mq0 * mq3;
		mq1mq1 = mq1 * mq1;
		mq1mq2 = mq1 * mq2;
		mq1mq3 = mq1 * mq3;
		mq2mq2 = mq2 * mq2;
		mq2mq3 = mq2 * mq3;
		mq3mq3 = mq3 * mq3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - mq2mq2 - mq3mq3) + my * (mq1mq2 - mq0mq3) + mz * (mq1mq3 + mq0mq2));
		hy = 2.0f * (mx * (mq1mq2 + mq0mq3) + my * (0.5f - mq1mq1 - mq3mq3) + mz * (mq2mq3 - mq0mq1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (mq1mq3 - mq0mq2) + my * (mq2mq3 + mq0mq1) + mz * (0.5f - mq1mq1 - mq2mq2));

		// Estimated direction of gravity and magnetic field
		halfvx = mq1mq3 - mq0mq2;
		halfvy = mq0mq1 + mq2mq3;
		halfvz = mq0mq0 - 0.5f + mq3mq3;
		halfwx = bx * (0.5f - mq2mq2 - mq3mq3) + bz * (mq1mq3 - mq0mq2);
		halfwy = bx * (mq1mq2 - mq0mq3) + bz * (mq0mq1 + mq2mq3);
		halfwz = bx * (mq0mq2 + mq1mq3) + bz * (0.5f - mq1mq1 - mq2mq2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * (1.0f * Cycle); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f * Cycle);
			integralFBz += twoKi * halfez * (1.0f * Cycle);
			gx += integralFBx; // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f * Cycle)); // pre-multiply common factors
	gy *= (0.5f * (1.0f * Cycle));
	gz *= (0.5f * (1.0f * Cycle));

	qa = mq0;
	qb = mq1;
	qc = mq2;
	mq0 += (-qb * gx - qc * gy - mq3 * gz);
	mq1 += (qa * gx + qc * gz - mq3 * gy);
	mq2 += (qa * gy - qb * gz + mq3 * gx);
	mq3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrtM(mq0 * mq0 + mq1 * mq1 + mq2 * mq2 + mq3 * mq3);
	mq0 *= recipNorm;
	mq1 *= recipNorm;
	mq2 *= recipNorm;
	mq3 *= recipNorm;

	angle[0] = atan2(2 * mq2 * mq3 + 2 * mq0 * mq1, -2 * mq1 * mq1 - 2 * mq2 * mq2 + 1) * MahonyAhrsAngle; // roll     -pi----pi
	angle[1] = asin(-2 * mq1 * mq3 + 2 * mq0 * mq2) * MahonyAhrsAngle;								   // pitch    -pi/2----pi/2
	angle[2] = atan2(2 * mq1 * mq2 + 2 * mq0 * mq3, -2 * mq2 * mq2 - 2 * mq3 * mq3 + 1) * MahonyAhrsAngle; // yaw      -pi----pi
	coefficient = (1.0f * Cycle) * recipNorm * MahonyAhrsAngle;

	return coefficient;
}

float MahonyAHRSupdateIMU(float *accel, float *gyro, float *angle, float Cycle)
{
	float coefficient;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float gx,gy,gz,ax,ay,az;
	ax = accel[0];
	ay = accel[1];
	az = accel[2];

	gx = gyro[0];
	gy = gyro[1];
	gz = gyro[2];

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = mq1 * mq3 - mq0 * mq2;
		halfvy = mq0 * mq1 + mq2 * mq3;
		halfvz = mq0 * mq0 - 0.5f + mq3 * mq3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx; // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f * Cycle)); // pre-multiply common factors
	gy *= (0.5f * (1.0f * Cycle));
	gz *= (0.5f * (1.0f * Cycle));
	qa = mq0;
	qb = mq1;
	qc = mq2;

	mq0 += (-qb * gx - qc * gy - mq3 * gz);
	mq1 += (qa * gx + qc * gz - mq3 * gy);
	mq2 += (qa * gy - qb * gz + mq3 * gx);
	mq3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrtM(mq0 * mq0 + mq1 * mq1 + mq2 * mq2 + mq3 * mq3);
	mq0 *= recipNorm;
	mq1 *= recipNorm;
	mq2 *= recipNorm;
	mq3 *= recipNorm;

	angle[0] = atan2(2 * mq2 * mq3 + 2 * mq0 * mq1, -2 * mq1 * mq1 - 2 * mq2 * mq2 + 1) * MahonyAhrsAngle; // roll     -pi----pi
	angle[1] = asin(-2 * mq1 * mq3 + 2 * mq0 * mq2) * MahonyAhrsAngle;								   // pitch    -pi/2----pi/2
	angle[2] = atan2(2 * mq1 * mq2 + 2 * mq0 * mq3, -2 * mq2 * mq2 - 2 * mq3 * mq3 + 1) * MahonyAhrsAngle; // yaw      -pi----pi
	coefficient = (1.0f * Cycle) * recipNorm * MahonyAhrsAngle;

	return coefficient;
}

float invSqrtM(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
