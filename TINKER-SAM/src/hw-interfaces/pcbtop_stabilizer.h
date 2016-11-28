/*
 * pcbtop_stabilizer.h
 *
 * Created: 13/11/2016 18:26:56
 *  Author: Badr
 */ 


#ifndef PCBTOP_STABILIZER_H_
#define PCBTOP_STABILIZER_H_


#include "../asf.h"
#include <string.h>
#include <stdlib.h>
#include "ctype.h"



#ifdef __cplusplus
extern "C" {
#endif





//-- PWM
static pwm_channel_t pwm_channel_instance;

#define TINKER_PWM_1 1
#define TINKER_PWM_2 2
#define TINKER_PWM_3 3
#define TINKER_PWM_4 4

#define PERIOD_VALUE       1000
#define PWM_FREQUENCY      400

void tinkerPwmInit(uint32_t pwmId);
void tinkerPwmWrite(uint32_t pwmId, uint32_t value);


//-- UART
#define TINKER_UART_BAUDE 115200

#define TINKER_UART_0 0
#define TINKER_UART_1 1
#define TINKER_UART_2 2
#define TINKER_UART_3 3
#define TINKER_UART_4 4

void tinkerUartInit(uint32_t uartId);
void tinkerUartGetChar(uint32_t uartId, uint8_t* data);
void tinkerUartPutChar(uint32_t uartId, const uint8_t c);
void tinkerUartPutBuffer(uint32_t uartId, const uint8_t *c, size_t size);
void tinkerUartPutString(uint32_t uartId, const char *str);


//-- I2C
#define TINKER_I2C_0 0

uint32_t tinkerInitI2C(uint8_t chip);
uint8_t tinkerReadI2C(uint8_t chip,uint8_t register);
//uint16_t tinkerReadI2C_16(uint8_t reg1,uint8_t reg2);
uint32_t tinkerWriteI2C(uint8_t chip, uint8_t reg, uint8_t val);

#endif /* PCBTOP_STABILIZER_H_ */

#ifdef __cplusplus
}
#endif