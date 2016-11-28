/*
 * TINKER-SAM.cpp
 *
 * Created: 15/11/2016 22:53:25
 * Author : Badr
 */ 


//#include "sam.h"


//int main(void)
//{
    ///* Initialize the SAM system */
    //SystemInit();
//
    ///* Replace with your application code */
    //while (1) 
    //{
    //}
//}

//#include "conf_board.h"
//#include "conf_clock.h"
//#include "conf_uart_serial.h"
//#include "conf_usb.h"

#include "ctype.h"

#include "misc/Helper.h"
#include "misc/PID_v1.h"

#include "hw-interfaces/pcbtop_stabilizer.h"
#include "hw-modules/TinkerUart.h"
#include "hw-modules/TinkerMPU6050.h"
#include "hw-modules/TinkerBMI160.h"
#include "hw-modules/DroniSensIMU11.h"
#include "hw-modules/TinkerPWM.h"
#include "services/DroniAttitude.h"

//extern int __ram_end__ = 0x20400000 + 0x00060000 - 1 ;

//--- UART
double _atof(char *s);
void _itoa(char*,int32_t);

static double _reqThrottle	= 0;
static double _reqRoll_x	= 0;
static double _reqPitch_y	= 0;
static double _reqYaw_z	= 0;

void parseAngles(char* d){
	
	char * pch;
	char buff1[20];strcpy(buff1,"0");
	char buff2[20];strcpy(buff2,"0");
	char buff3[20];strcpy(buff3,"0");
	char buff4[20];strcpy(buff4,"0");

	
	//TinkerUart0.println(d);
	int i = 0;
	char c = d[i];
	char _p[2];_p[0]=d[i];_p[1]='\0';
	while(c != '\0' && i<100){
		_p[0]=d[i];
		strcat(buff1,_p);i++;
		c=d[i];
		if(c==' '){
			i++;c=d[i];
			break;
		}
	}
	while(c != '\0' && i<100){
		_p[0]=d[i];
		strcat(buff2,_p);i++;
		c=d[i];
		if(c==' '){
			i++;c=d[i];
			break;
		}
	}
	while(c != '\0' && i<100){
		_p[0]=d[i];
		strcat(buff3,_p);i++;
		c=d[i];
		if(c==' '){
			i++;c=d[i];
			break;
		}
	}
	while(c != '\0' && i<100){
		_p[0]=d[i];
		strcat(buff4,_p);i++;
		c=d[i];
		if(c==' '){
			i++;c=d[i];
			break;
		}
	}
	//TinkerUart0.println(buff1);
	//TinkerUart0.println(buff2);
	//TinkerUart0.println(buff3);
	//TinkerUart0.println(buff4);
	
	
	//--
	
	_reqThrottle = atof(buff1);	
	_reqRoll_x	 = atof(buff2);
	_reqPitch_y	 = atof(buff3);
	_reqYaw_z	 = atof(buff4);
	return;
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqThrottle = _atof(buff1);
	_reqRoll_x	 = _atof(buff2);
	_reqPitch_y	 = _atof(buff3);
	_reqYaw_z	 = _atof(buff4);
	_reqYaw_z	 = atof(buff4);

	
//
	



}


static char _data[256];
static uint32_t _data_index = 0;
static bool _data_start = false;
static uint8_t c = 0;
void UART0_Handler(void){
	uint32_t status = UART0->UART_SR;
	usart_serial_getchar((usart_if)UART0,&c);
	
	
	if(_data_index>255) 
		_data_index = 0;
	
	TinkerUart0.writeln(c);
	
	if(c=='{'){
			_data_start = true;
	}else if( c == '}'){
			_data[_data_index] = '\0';
			_data_index = 0;
			if(!_data_start){						
				
			}else{
				_data_start = false;
				parseAngles(_data);
			}
	}else{
			if(_data_start && (isdigit(c) || c==' ' || c=='.')){
				_data[_data_index] = c;
				_data_index ++;
			}
	}

	// Acknowledge errors
	if (status & (UART_SR_OVRE | UART_SR_FRAME))
	{
		UART0->UART_CR |= UART_CR_RSTSTA;
	}	
}

//------
//-- pid
static double targetAngX = 0, pidAngX = 0, estimAngleX = 0;
static double targetAngY = 0, pidAngY = 0, estimAngleY = 0;
static double targetAngZ = 0, pidAngZ = 0, estimAngleZ = 0;


//---PWM
static TinkerPWM	motor1(TINKER_PWM_1);
static TinkerPWM	motor2(TINKER_PWM_2);
static TinkerPWM	motor3(TINKER_PWM_3);
static TinkerPWM	motor4(TINKER_PWM_4);

//-- LOOPs
static uint32_t main_loop_timestamp = 0;
static uint32_t main_loop_period = 5000; //period of 5ms for the main loop
static int32_t main_loop_delay = 0;

//-- Droni Attitude Estimation
DroniAttitude droniAtt;
static uint32_t task1_timestamp = 0;
static uint32_t task1_period = 2000; //period of 2ms
static uint32_t task1_duration = 0;
void task1_estimateAttitude(void){
	
	uint32_t now = micros();
	if(micros() < task1_timestamp + task1_period) return;

	//--
	droniAtt.step(((double)(now - task1_timestamp))/1000000);

	//--
	task1_timestamp = now;
}

//----
static bool canReadTargetAngles = true;
void task4_update_target_angles(void){
	
	
	canReadTargetAngles = false;
	if(_reqRoll_x>-30 && _reqRoll_x<30)
	targetAngX=_reqRoll_x;
	
	if(_reqPitch_y>-30 && _reqPitch_y<30)
	targetAngY=_reqPitch_y;
	
	if(_reqYaw_z>-30 && _reqYaw_z<30)
	targetAngZ=_reqYaw_z;
	canReadTargetAngles = true;

}
 void TinkerUart0_printlnNumber(int32_t i);
//-- traces
static uint32_t task0_timestamp = 0;
static uint32_t task0_period = 1000000; //period of 1 sec
extern int _sstack, _estack;
extern int _sfixed, _efixed;
extern int _etext;
extern int _srelocate, _erelocate;
extern int _szero, _ezero;

void task0_traces(void){
	
	//--
	if(micros() < task0_timestamp + task0_period) return;

	//--
	TinkerUart0.println("-----------------");
	TinkerUart0.print("_sstack:"); TinkerUart0.printlnNumber((uint32_t)&_estack - (uint32_t)&_sstack);
	//TinkerUart0_println("-----------------");
	//TinkerUart0_printlnNumber(99);
	//TinkerUart0_printlnNumber(99);
	//TinkerUart0_printlnNumber(99);
	//TinkerUart0_printlnNumber(99);
	//TinkerUart0_printlnNumber(99);
	//TinkerUart0_printlnNumber(99);
	//TinkerUart0_printFloat(1234.6,10);
	//TinkerUart0.print("gyroXangle:"); TinkerUart0.printlnDouble(droniAtt.gyroXangle);
	//TinkerUart0.print("gyroYangle:"); TinkerUart0.printlnDouble(droniAtt.gyroYangle);
	//TinkerUart0.print("compAngleX:"); TinkerUart0.printlnDouble(droniAtt.compAngleX);
	//TinkerUart0.print("compAngleY:"); TinkerUart0.printlnDouble(droniAtt.compAngleY);
	TinkerUart0.print("estimAngleX:"); TinkerUart0.printlnDouble(estimAngleX);
	TinkerUart0.print("estimAngleY:"); TinkerUart0.printlnDouble(estimAngleY);
	TinkerUart0.println("-----------------");
	//TinkerUart0.print("AccelX:"); TinkerUart0.printlnDouble(droniAtt.accX);
	//TinkerUart0.print("AccelY:"); TinkerUart0.printlnDouble(droniAtt.accY);
	//TinkerUart0.print("AccelZ:"); TinkerUart0.printlnDouble(droniAtt.accZ);
	//TinkerUart0.println("-----------------");
	//TinkerUart0.print("GyroX:"); TinkerUart0.printlnDouble(droniAtt.gyroX);
	//TinkerUart0.print("GyroY:"); TinkerUart0.printlnDouble(droniAtt.gyroY);
	//TinkerUart0.print("GyroZ:"); TinkerUart0.printlnDouble(droniAtt.gyroZ);
	TinkerUart0.println("-----------------");
	TinkerUart0.print("pidAngX:"); TinkerUart0.printlnDouble(pidAngX);
	TinkerUart0.print("pidAngY:"); TinkerUart0.printlnDouble(pidAngY);
	TinkerUart0.println("-----------------");
	TinkerUart0.print("Estim time us (3100):"); TinkerUart0.printlnDouble(task1_duration);
	TinkerUart0.println("-----------------");
////
	TinkerUart0.print("_reqThrottle:");
	TinkerUart0.printlnDouble(_reqThrottle);
////
	TinkerUart0.print("_reqRoll_x:");
	TinkerUart0.printlnDouble(_reqRoll_x);
////
	TinkerUart0.print("_reqPitch_y:");
	TinkerUart0.printlnDouble(_reqPitch_y);
	TinkerUart0.print("_reqYaw_z:");
	TinkerUart0.printlnDouble(_reqYaw_z);
	TinkerUart0.println("-----------------");

	//udi_cdc_putc('A');
	

	//mdelay(1000000);
	//--
	task0_timestamp = micros();
}



//-----
static uint32_t task2_timestamp = 0;
static uint32_t task2_period = 5000; 
static double ITermX = 0, ITermY = 0;
static double lastInputX = 0,lastInputY = 0;
static double kp=2, ki=5 * task2_period/1000000, kd=1/ task2_period*1000000;
static double outMax = 100, outMin = -100;
void task2_compute(void)
{
	unsigned long now = micros();
	unsigned long timeChange = (now - task2_timestamp);

	task4_update_target_angles();

	if(timeChange>=task2_period)
	{		
		/*Compute all the working error variables*/
		double input_x = estimAngleX;
		double input_y = estimAngleY;

		double error_x = targetAngX - input_x;
		double error_y = targetAngY - input_y;

		ITermX+= (ki * error_x);
		ITermY+= (ki * error_y);

		if(ITermX > outMax) ITermX= outMax;
		else if(ITermX < outMin) ITermX= outMin;

		if(ITermY > outMax) ITermY= outMax;
		else if(ITermY < outMin) ITermY= outMin;

		double dInputX = (input_x - lastInputX);
		double dInputY = (input_y - lastInputY);
		
		/*Compute PID Output*/
		double outputX = kp * error_x + ITermX- kd * dInputX;
		double outputY = kp * error_y + ITermY- kd * dInputY;
		
		if(outputX > outMax) outputX = outMax;
		else if(outputX < outMin) outputX = outMin;
		pidAngX = outputX;

		if(outputY > outMax) outputY = outMax;
		else if(outputY < outMin) outputY = outMin;
		pidAngY = outputY;
		
		/*Remember some variables for next time*/
		lastInputX = input_x;
		lastInputY = input_y;

		task2_timestamp = now;
	}
	
}

//---
static uint32_t task3_timestamp = 0;
static uint32_t task3_period = 10000;
static uint32_t motorA = 0;
static uint32_t motorB = 0;
static uint32_t motorC = 0;
static uint32_t motorD = 0;
void task3_control_motors(void){
	uint32_t yawpid = 0;

	motorA = _reqThrottle + (int32_t) ( pidAngX - pidAngY) - yawpid;
	motorB = _reqThrottle + (int32_t) (-pidAngX - pidAngY) + yawpid;
	motorC = _reqThrottle + (int32_t) (-pidAngX + pidAngY) - yawpid;
	motorD = _reqThrottle + (int32_t) ( pidAngX + pidAngY) + yawpid;

	motor1.write(motorA);
	motor2.write(motorB);
	motor3.write(motorC);
	motor4.write(motorD);
}




int main (void)
{

	/* Initialize the SAM system */
	//wdt_disable(WDT); --> we will use the watch dog for a period of 10ms
	sysclk_init();
	board_init();
	ioport_init();
	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();
	fpu_enable();
	// disable JTAG
	MATRIX->CCFG_SYSIO |= (1 << 4) | (1 << 5);//to use pin PB5 and PB4

	/* Configure systick for 1 µs */
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000000)) {
	}

	//---SETUP

	//- init CDC USB
	//irq_initialize_vectors();
	//cpu_irq_enable();
	irq_register_handler(UART0_IRQn,1);
	uart_enable_interrupt(UART0, UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME);
	//udc_start();

	//- init PWM
	//--> init done at static declaration of motors
	
	//- init UART
	TinkerUart0.init();

	//- init I2C
	uint8_t status = tinkerInitI2C(0x00);
	if(status != 0){
		TinkerUart0.print("error init I2C:");
		TinkerUart0.printNumber(status);
		while(1);
	}
	TinkerUart0.println("Init I2C OK");


	//- init Attitude estimator with DroniSens 1.1	
	uint32_t timer = micros();
	droniAtt.init();
	mdelay(100000);//--> delay to let the sensors to initialize and power up
	TinkerUart0.println("Init droniAtt OK");

	//- PID

	//PID myPID_angleX(&estimAngleX, &pidAngX, &targetAngX, Kp, Ki, Kd, DIRECT);
	//PID myPID_angleY(&estimAngleY, &pidAngY, &targetAngY, Kp, Ki, Kd, DIRECT);
		//myPID_angleX.SetMode(AUTOMATIC);
		//myPID_angleY.SetMode(AUTOMATIC);
	//---LOOP	
	while (1) {
		if(micros() < main_loop_timestamp + main_loop_period) continue;

		//--
		uint32_t start = micros();
		task1_estimateAttitude();
		estimAngleX = droniAtt.kalAngleX;
		estimAngleY = droniAtt.kalAngleY;
		

	
		//--
		task0_traces();
		
		//TinkerUart0.print("$");
		//TinkerUart0.printNumber((micros()));
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble((estimAngleX));
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble((pidAngX));
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble((estimAngleY));
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble((pidAngY));
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble(motorA);
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble(motorB);
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble(motorC);
		//TinkerUart0.print(" ");
		//TinkerUart0.printDouble(motorD);
		//TinkerUart0.print(";");

		//--
		task2_compute();
		
		//--
		task3_control_motors();
		
		//--
		uint32_t end = micros();
		task1_duration = end - start;

		main_loop_timestamp = micros();
		//mdelay(1000000);
		
		
		wdt_restart(WDT);

	}


}




//--
//uint8_t add = 0x69;
//TinkerBMI160 bmi(add);
//bmi.init();
//
//
//
//tinkerWriteI2C(add,BMI160_RA_ACCEL_RANGE,BMI160_ACCEL_RANGE_2G);
//mdelay(100000);
//
//bmi.update();
//TinkerUart0.printlnNumber(tinkerReadI2C(add,BMI160_ACC_X_H_G)<<8|tinkerReadI2C(add,BMI160_ACC_X_L_G));
//TinkerUart0.printlnNumber(tinkerReadI2C(add,0x00));
//while(1);