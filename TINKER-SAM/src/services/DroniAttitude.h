	/*
 * DroniAttitude.h
 *
 * Created: 19/11/2016 17:01:22
 *  Author: Badr
 */ 


#ifndef DRONIATTITUDE_H_
#define DRONIATTITUDE_H_

#include "../hw-modules/DroniSensIMU11.h"
#include "../misc/Kalman.h"
//#include "math.h" // new math??


#define RAD_TO_DEG	57.295779513082320876798154814105
#define  RESTRICT_PITCH

class DroniAttitude{
	Kalman kalmanX; // Create the Kalman instances
	Kalman kalmanY;

	public:
	/* IMU Data */
	double accX, accY, accZ;
	double gyroX, gyroY, gyroZ;
	
	double gyroXangle, gyroYangle; // Angle calculate using the gyro only
	double compAngleX, compAngleY; // Calculated angle using a complementary filter
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter


	private:
		DroniSensIMU11 _droniSens;

	public:
		void init(void){
			_droniSens.init();
			_droniSens.update();
			
			//--
			accX	= _droniSens.getAccelX();
			accY	= _droniSens.getAccelY();
			accZ	= _droniSens.getAccelZ();
			gyroX	= _droniSens.getGyroX();
			gyroY	= _droniSens.getGyroY();
			gyroZ	= _droniSens.getGyroZ();

			//mdelay(10000);

			#ifdef RESTRICT_PITCH // Eq. 25 and 26
				double roll  = atan2(accY, accZ) * RAD_TO_DEG;
				double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
			#else // Eq. 28 and 29
				double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
				double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
			#endif

			kalmanX.setAngle(roll); // Set starting angle
			kalmanY.setAngle(pitch);
			gyroXangle = roll;
			gyroYangle = pitch;
			compAngleX = roll;
			compAngleY = pitch;

		}

		void step(double dt){
			
			_droniSens.update();

			//--
			accX	= _droniSens.getAccelX();
			accY	= _droniSens.getAccelY();
			accZ	= _droniSens.getAccelZ();
			gyroX	= _droniSens.getGyroX();
			gyroY	= _droniSens.getGyroY();
			gyroZ	= _droniSens.getGyroZ();

			//--
			#ifdef RESTRICT_PITCH // Eq. 25 and 26
				double roll  = atan2(accY, accZ) * RAD_TO_DEG;
				double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
			#else // Eq. 28 and 29
				double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
				double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
			#endif

			//DO I NEED IT?
			double gyroXrate = gyroX;// / 131.0; // Convert to deg/s
			double gyroYrate = gyroY;// / 131.0; // Convert to deg/s

			#ifdef RESTRICT_PITCH
				// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
				if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
					kalmanX.setAngle(roll);
					compAngleX = roll;
					kalAngleX = roll;
					gyroXangle = roll;
				} else
				kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

				if (abs(kalAngleX) > 90)
				gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
				kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
			#else
				// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
				if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
					kalmanY.setAngle(pitch);
					compAngleY = pitch;
					kalAngleY = pitch;
					gyroYangle = pitch;
				} else
				kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

				if (abs(kalAngleY) > 90)
				gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
				kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
			#endif

			gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
			gyroYangle += gyroYrate * dt;
			//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
			//gyroYangle += kalmanY.getRate() * dt;

			compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
			compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

			// Reset the gyro angle when it has drifted too much
			if (gyroXangle < -180 || gyroXangle > 180)
				gyroXangle = kalAngleX;
			if (gyroYangle < -180 || gyroYangle > 180)
				gyroYangle = kalAngleY;
		}
		
};



#endif /* DRONIATTITUDE_H_ */