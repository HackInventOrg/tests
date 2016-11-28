/*
 * DroniSensIMU11.h
 *
 * Created: 19/11/2016 13:26:02
 *  Author: Badr
 */ 

#include "TinkerBMI160.h"
class TinkerBMI160;

#ifndef DRONISENSIMU11_H_
#define DRONISENSIMU11_H_


class DroniSensIMU11 {
	private:
		TinkerBMI160 _bmi1; //--> for low acceleration detection
		TinkerBMI160 _bmi2; //--> for high acceleration detection

	public:
		DroniSensIMU11():_bmi1(0x68),_bmi2(0x69){

		}

		void init(void){
			//--
			_bmi2.init();
			_bmi1.init();
			//--
			_bmi1.setAccelRange(BMI160_ACCEL_RANGE_2G);
			_bmi1.setGyroRange(BMI160_GYRO_RANGE_250);
			//--
			_bmi2.setAccelRange(BMI160_ACCEL_RANGE_8G);
			_bmi2.setGyroRange(BMI160_GYRO_RANGE_1000);
		}

		void update(void){
			_bmi1.update();
			_bmi2.update();
		}

		double getAccelX(void){
			return _bmi2.accel_x > BMI160_2G ? _bmi2.accel_x : (0.2*_bmi2.accel_x + 0.8*_bmi1.accel_x) ;
		}
		double getAccelY(void){
			return _bmi2.accel_y > BMI160_2G ? _bmi2.accel_y : (0.2*_bmi2.accel_y + 0.8*_bmi1.accel_y) ;
		}
		double getAccelZ(void){
			return _bmi2.accel_z > BMI160_2G ? _bmi2.accel_z : (0.2*_bmi2.accel_z + 0.8*_bmi1.accel_z) ;
		}

		double getGyroX(void){
			return _bmi2.gyro_x > BMI160_250DEG ? _bmi2.gyro_x : (0.2*_bmi2.gyro_x + 0.8*_bmi1.gyro_x) ;
		}
		double getGyroY(void){
			return _bmi2.gyro_y > BMI160_250DEG ? _bmi2.gyro_y : (0.2*_bmi2.gyro_y + 0.8*_bmi1.gyro_y) ;
		}
		double getGyroZ(void){
			return _bmi2.gyro_z > BMI160_250DEG ? _bmi2.gyro_z : (0.2*_bmi2.gyro_z + 0.8*_bmi1.gyro_z) ;
		}

};




#endif /* DRONISENSIMU11_H_ */