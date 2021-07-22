// Calculate position from MPU data

#include "MPUcalc.h"
#include "MPU9250.h"


void MPU_CalculateFromRAWData(struct MPUstr* d,float timedelta)
{

	float AccelVectorPitch;
	float AccelVectorRoll;
	float p,q,r;
	float X,Y,Z;

	//Offset RAW gyro values with calibrated offsets
	d->Gyroscope_X_Cal = (float)(d->Gyroscope_X_RAW) - d->Offset_Gyro_X;
	d->Gyroscope_Y_Cal = (float)(d->Gyroscope_Y_RAW) - d->Offset_Gyro_Y;
	d->Gyroscope_Z_Cal = (float)(d->Gyroscope_Z_RAW) - d->Offset_Gyro_Z;


	//GYRO AND ACCEL DATA in STANDARD X,Y,Z directions Roll (nose), Pitch(right wing), Yaw (down)
	//Sensor MPU 6050 axis position X (right wing), Y (nose), Z (up)
	d->Gyro_X = d->Gyroscope_Y_Cal;
	d->Gyro_Y = d->Gyroscope_X_Cal;
	d->Gyro_Z = -d->Gyroscope_Z_Cal;

	d->Accel_X = d->Accelerometer_Y_RAW;
	d->Accel_Y = d->Accelerometer_X_RAW;
	d->Accel_Z = -d->Accelerometer_Z_RAW;


	//Accelerometer angles-----------------------------------------------------------------
	AccelVectorRoll =  sqrt( (d->Accel_X * d->Accel_X) + (d->Accel_Z * d->Accel_Z) );
	AccelVectorPitch = sqrt( (d->Accel_Y * d->Accel_Y) + (d->Accel_Z * d->Accel_Z) );

	d->Angle_Accel_Roll  = -atan(d->Accel_Y/AccelVectorRoll) * RADIANSTODEGREES;
	d->Angle_Accel_Pitch = atan(d->Accel_X/AccelVectorPitch) * RADIANSTODEGREES;

	//Compensate offset with spirit level manual offset
	d->Angle_Accel_Pitch-=ACCELPITCHMANUALOFFSET;
	d->Angle_Accel_Roll-=ACCELROLLMANUALOFFSET;
	//Save angles in Radians
	d->Angle_Accel_Pitch_Rad=d->Angle_Accel_Pitch*DEGREESTORADIANS;
	d->Angle_Accel_Roll_Rad=d->Angle_Accel_Roll*DEGREESTORADIANS;

	//Calculate angular gyro velocities----------------------------------------------------
	d->AngleSpeed_Gyro_X = d->Gyro_X / GYROCONSTANT;
	d->AngleSpeed_Gyro_Y = d->Gyro_Y / GYROCONSTANT;
	d->AngleSpeed_Gyro_Z = d->Gyro_Z / GYROCONSTANT;

	//convert angular velocity to radians/s
	p = d->AngleSpeed_Gyro_X * DEGREESTORADIANS;
	q = d->AngleSpeed_Gyro_Y * DEGREESTORADIANS;
	r = d->AngleSpeed_Gyro_Z * DEGREESTORADIANS;

	//Save Angles in radians from previous STEP
	X = d->Roll_Rad;
	Y = d->Pitch_Rad;
	Z = d->Angle_Gyro_Yaw_Rad ;

	//TRANSFORM gyro data to Euler Angles with complementary filter with accelerometer
	d->Roll_Rad   = 0.999 * (X + timedelta * (p  +  q*sin(X)*tan(Y) + r*cos(X)*tan(Y) ) ) + 0.001*d->Angle_Accel_Roll_Rad;
	d->Pitch_Rad  = 0.999 * (Y + timedelta * (q * cos(X) -  r * sin(X) ) 			    ) + 0.001*d->Angle_Accel_Pitch_Rad;
	d->Angle_Gyro_Yaw_Rad = Z + timedelta * (q*sin(X)/cos(Y) + r*cos(X)/cos(Y) ); //Only Gyro Angle will drift

	//Convert to Degrees
	d->Roll   = d->Roll_Rad * RADIANSTODEGREES;
	d->Pitch  = d->Pitch_Rad * RADIANSTODEGREES;
	d->Angle_Gyro_Yaw   = d->Angle_Gyro_Yaw_Rad * RADIANSTODEGREES;
}

void GetGyroOffset(struct MPUstr* d, int32_t Loops, uint32_t Delayms)
{
	int32_t SUMGyroX,SUMGyroY,SUMGyroZ;
	uint32_t i;

	SUMGyroX=0;
	SUMGyroY=0;
	SUMGyroZ=0;

	for(i=0;i<Loops;i++)
	{

		MPU9250_GetGyroData(&mpuDataStr);

		  SUMGyroX+=d->Gyroscope_X_RAW;
		  SUMGyroY+=d->Gyroscope_Y_RAW;
		  SUMGyroZ+=d->Gyroscope_Z_RAW;

		  HAL_Delay(Delayms);

	}

	d->Offset_Gyro_X=(float)(SUMGyroX) / (float)(Loops);
	d->Offset_Gyro_Y=(float)(SUMGyroY) / (float)(Loops);
	d->Offset_Gyro_Z=(float)(SUMGyroZ) / (float)(Loops);


	MPU9250_GetAccelData(&mpuDataStr);

	MPU_CalculateFromRAWData(&mpuDataStr,0); //Gyro angles don't matter

	//Transfer accelerometer angles to Gyro
	d->Pitch = d->Angle_Accel_Pitch;
	d->Roll =  d->Angle_Accel_Roll;

	d->Angle_Gyro_Pitch_Rad = d->Angle_Accel_Pitch_Rad;
	d->Angle_Gyro_Roll_Rad = d->Angle_Accel_Roll_Rad;

	d->Angle_Gyro_Yaw = 0;
	d->Angle_Gyro_Yaw_Rad = 0;
}


