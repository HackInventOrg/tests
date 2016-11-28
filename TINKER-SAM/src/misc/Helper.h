/*
 * Helper.h
 *
 * Created: 21/11/2016 20:41:38
 *  Author: Badr
 */ 


#ifndef HELPER_H_
#define HELPER_H_

#include "same70.h"
#include "ctype.h"

void SysTick_Handler(void);
void mdelay(uint32_t ul_dly_ticks);
uint32_t micros(void);

double _atof(char *s);
void _itoa(char*,int32_t);


#endif /* HELPER_H_ */