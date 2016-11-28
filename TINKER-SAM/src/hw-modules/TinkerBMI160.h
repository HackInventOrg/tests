/*
 * TINKERBMI160.h
 *
 * Created: 17/11/2016 02:10:02
 *  Author: Badr
 */ 


#ifndef TINKERBMI160_H_
#define TINKERBMI160_H_

#include "TinkerIMU.h"

//#define BMI160_MAG_X_L_G       0x04 --> not used, only accel and gyro
//#define BMI160_MAG_X_H_G       0x05
//#define BMI160_MAG_Y_L_G       0x06
//#define BMI160_MAG_Y_H_G       0x07
//#define BMI160_MAG_Z_L_G       0x08
//#define BMI160_MAG_Z_H_G       0x09
//#define BMI160_RHALL_L_G       0x0a
//#define BMI160_RHALL_H_G       0x0b
#define BMI160_GYR_X_L_G       0x0c
#define BMI160_GYR_X_H_G       0x0d
#define BMI160_GYR_Y_L_G       0x0e
#define BMI160_GYR_Y_H_G       0x0f
#define BMI160_GYR_Z_L_G       0x10
#define BMI160_GYR_Z_H_G       0x11
#define BMI160_ACC_X_L_G       0x12
#define BMI160_ACC_X_H_G       0x13
#define BMI160_ACC_Y_L_G       0x14
#define BMI160_ACC_Y_H_G       0x15
#define BMI160_ACC_Z_L_G       0x16
#define BMI160_ACC_Z_H_G       0x17

#define BMI160_RA_ACCEL_CONF        0X40
#define BMI160_RA_ACCEL_RANGE		0X41

#define BMI160_RA_GYRO_CONF         0X42
#define BMI160_RA_GYRO_RANGE		0X43

typedef enum {
	BMI160_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
	BMI160_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
	BMI160_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
	BMI160_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
} BMI160AccelRange;

typedef enum {
	BMI160_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
	BMI160_GYRO_RANGE_1000 = 1,     /**<  +/- 1000 degrees/second */
	BMI160_GYRO_RANGE_500  = 2,      /**<  +/-  500 degrees/second */
	BMI160_GYRO_RANGE_250  = 3,      /**<  +/-  250 degrees/second */
	BMI160_GYRO_RANGE_125  = 4,      /**<  +/-  125 degrees/second */
} BMI160GyroRange;

#define BMI160_2G		2
#define BMI160_8G		8
#define BMI160_250DEG	250
#define BMI160_1000DEG	1000

#define MAX_UINT16	32767

#include "misc/Helper.h"

class TinkerBMI160 : public TinkerIMU{
	
	private:
		double _accel_range = 2;
		double _gyro_range = 250;

	public:
		TinkerBMI160(uint8_t add):TinkerIMU(add){			

			return;
		};

		void init(void){
			tinkerWriteI2C(_address, 0x6D, 0b11111); // --> run auto test
			//mdelay(100000);
			//TinkerUart0.printlnNumber(tinkerReadI2C(add,0x1B));
			mdelay(100000);
			tinkerWriteI2C(_address,0x7E,0xB6); //--> soft reset
			mdelay(100000);
			tinkerReadI2C(_address,0x7F);
			mdelay(100000);
			tinkerWriteI2C(_address,0x7E,0x11); //--> Set PMU mode of accelerometer to normal
			mdelay(100000);
			tinkerWriteI2C(_address,0x7E,0x15); //--> Set PMU mode of gyroscope to normal
			mdelay(100000);
			return;
		}

		void setAccelRange(BMI160AccelRange range){
			if(range == BMI160_ACCEL_RANGE_2G)
				_accel_range = 2;
			else if(range == BMI160_ACCEL_RANGE_4G)
				_accel_range = 4;
			else if(range == BMI160_ACCEL_RANGE_8G)
				_accel_range = 8;
			else if(range == BMI160_ACCEL_RANGE_16G)
				_accel_range = 16;
			else
				_accel_range = 2;
			tinkerWriteI2C(_address,BMI160_RA_ACCEL_RANGE,range);
		}
		void setGyroRange(BMI160GyroRange range){
			if(range == BMI160_GYRO_RANGE_125)
				_gyro_range = 125;
			else if(range == BMI160_GYRO_RANGE_500)
				_gyro_range = 500;
			else if(range == BMI160_GYRO_RANGE_250)
				_gyro_range = 250;
			else if(range == BMI160_GYRO_RANGE_1000)
				_gyro_range = 1000;
			else if(range == BMI160_GYRO_RANGE_2000)
				_gyro_range = 2000;
			else
				_gyro_range = 125;
			tinkerWriteI2C(_address,BMI160_RA_GYRO_RANGE,range);
		}

		void update(void){
			accel_x = (int16_t)(tinkerReadI2C(_address,BMI160_ACC_X_H_G)<<8|tinkerReadI2C(_address,BMI160_ACC_X_L_G))
				*_accel_range
				/(double)MAX_UINT16;
			accel_y = (int16_t)(tinkerReadI2C(_address,BMI160_ACC_Y_H_G)<<8|tinkerReadI2C(_address,BMI160_ACC_Y_L_G))
				*_accel_range
				/(double)MAX_UINT16;
			accel_z = (int16_t)(tinkerReadI2C(_address,BMI160_ACC_Z_H_G)<<8|tinkerReadI2C(_address,BMI160_ACC_Z_L_G))
				*_accel_range
				/(double)MAX_UINT16;

			gyro_x = (int16_t)(tinkerReadI2C(_address,BMI160_GYR_X_H_G)<<8|tinkerReadI2C(_address,BMI160_GYR_X_L_G))
				*_gyro_range
				/(double)MAX_UINT16;
			gyro_y = (int16_t)(tinkerReadI2C(_address,BMI160_GYR_Y_H_G)<<8|tinkerReadI2C(_address,BMI160_GYR_Y_L_G))
				*_gyro_range
				/(double)MAX_UINT16;
			gyro_z = (int16_t)(tinkerReadI2C(_address,BMI160_GYR_Z_H_G)<<8|tinkerReadI2C(_address,BMI160_GYR_Z_L_G))
				*_gyro_range
				/(double)MAX_UINT16;
		};

};



#endif /* TINKERBMI160_H_ */