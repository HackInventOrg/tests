/*
 * TinkerMPU6050.h
 *
 * Created: 17/11/2016 02:10:02
 *  Author: Badr
 */ 


#ifndef TINKERMPU6050_H_
#define TINKERMPU6050_H_

#include "TinkerIMU.h"

 #define power_mgmt 0x6b
 #define address 0x68 // Achtung 7Bit Addresse wird in der ic2 lib geshifted


 //define gyro register
 #define gyro_x_H 0x43
 #define gyro_x_L 0x44
 #define gyro_y_H 0x45
 #define gyro_y_L 0x46
 #define gyro_z_H 0x47
 #define gyro_z_L 0x48

 //define accel register
 #define accel_x_H 0x3b
 #define accel_x_L 0x3c
 #define accel_y_H 0x3d
 #define accel_y_L 0x3e
 #define accel_z_H 0x3f
 #define accel_z_L 0x40

 //define temp register
 #define temp_H 0x41
 #define temp_L 0x42

class TinkerMPU6050 : public TinkerIMU{
	
	public:
		TinkerMPU6050(uint8_t add):TinkerIMU(add){
			return;
		};
		void update(void){
			accel_x = tinkerReadI2C(_address,accel_x_H)<<8|tinkerReadI2C(_address,accel_x_L);
		};

};



#endif /* TINKERMPU6050_H_ */