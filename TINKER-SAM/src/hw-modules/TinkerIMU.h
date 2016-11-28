/*
 * TinkerUart.h
 *
 * Created: 15/11/2016 23:43:56
 *  Author: Badr
 */ 


#ifndef TINKERIMU_H_
#define TINKERIMU_H_

#include "conf_board.h"



class TinkerIMU {//We use only TWIHS 1
	
	protected:
		uint8_t _address = 0x00;

	public:
		double accel_x = 0;
		double accel_y = 0;
		double accel_z = 0;

		double gyro_x = 0;
		double gyro_y = 0;
		double gyro_z = 0;

		double mag_x = 0;
		double mag_y = 0;
		double mag_z = 0;

	public:
		TinkerIMU(uint8_t address){_address = address;}

		
		
 };


#endif /* TINKERIMU_H_ */