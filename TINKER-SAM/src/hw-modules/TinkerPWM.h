/*
 * TinkerPWM.h
 *
 * Created: 20/11/2016 21:11:42
 *  Author: Badr
 */ 


#ifndef TINKERPWM_H_
#define TINKERPWM_H_

#include "conf_board.h"

class TinkerPWM{
	private:
		uint32_t _pwmId = TINKER_PWM_1;
	public:
		TinkerPWM(uint32_t pwmId){
			_pwmId = pwmId;
			tinkerPwmInit(pwmId);
		}

		void init(uint32_t pwmId){
			tinkerPwmInit(pwmId);
		}

		void write(uint32_t duty){
			tinkerPwmWrite(_pwmId, duty);
		}

};




#endif /* TINKERPWM_H_ */