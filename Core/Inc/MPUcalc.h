#ifndef MPUCALC_H_
#define MPUCALC_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define  PI (float)(3.1415926535897932384626433832795)

struct MPUstr {
	int16_t Accelerometer_X_RAW;
	int16_t Accelerometer_Y_RAW;
	int16_t Accelerometer_Z_RAW;

	int16_t Gyroscope_X_RAW;
	int16_t Gyroscope_Y_RAW;
	int16_t Gyroscope_Z_RAW;

	int16_t Mag_X_RAW;
	int16_t Mag_Y_RAW;
	int16_t Mag_Z_RAW;

	float Offset_Gyro_X;
	float Offset_Gyro_Y;
	float Offset_Gyro_Z;

	float Gyroscope_X_Cal;
	float Gyroscope_Y_Cal;
	float Gyroscope_Z_Cal;

	//GYRO AND ACCEL AND MAG DATA in STANDARD X,Y,Z directions Roll (nose), Pitch(right wing), Yaw (down)
	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;

	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;

	int16_t Mag_X;
	int16_t Mag_Y;
	int16_t Mag_Z;

	float Angle_Accel_Pitch;
	float Angle_Accel_Roll;
	float Angle_Accel_Yaw;

	float Angle_Accel_Pitch_Rad;
	float Angle_Accel_Roll_Rad;

	float Angle_Gyro_Pitch;
	float Angle_Gyro_Roll;
	float Angle_Gyro_Yaw;

	float Angle_Gyro_Pitch_Rad;
	float Angle_Gyro_Roll_Rad;
	float Angle_Gyro_Yaw_Rad;

	float AngleSpeed_Gyro_X;
	float AngleSpeed_Gyro_Y;
	float AngleSpeed_Gyro_Z;

	float Pitch;
	float Roll;
	float Yaw;

	float Pitch_Rad;
	float Roll_Rad;
	float Yaw_Rad;

} ;

struct Quaternions
{
	float w;
	float x;
	float y;
	float z;
};

struct GravityVector
{
	float x;
	float y;
	float z;
};

struct Angles
{
	float yaw;
	float pitch;
	float roll;
};


void CalculateQuaternions(struct Quaternions *q, uint8_t *fifodata);
void CalculateGravityVector(struct Quaternions *q, struct GravityVector *v);
void CalculateYawPitchRoll(struct Quaternions *q, struct GravityVector *v, struct Angles *ang);
void MPU_CalculateFromRAWData(struct MPUstr* d,float timedelta);
void GetGyroOffset(struct MPUstr* d, int32_t Loops, uint32_t Delayms);

#endif /* MPU-CALC_H_ */
