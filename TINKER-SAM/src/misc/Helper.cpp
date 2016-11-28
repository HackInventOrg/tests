/*
 * Heloper.cpp
 *
 * Created: 21/11/2016 22:28:19
 *  Author: Badr
 */ 

 #include "Helper.h"
 #include "asf.h"
 //#include "stdio_serial.h"
 #include "conf_board.h"
 #include "conf_clock.h"

 volatile uint32_t g_ul_us_ticks = 0;

 void SysTick_Handler(void){
	 g_ul_us_ticks++;
 }

 void mdelay(uint32_t ul_dly_ticks){
	 uint32_t ul_cur_ticks;

	 ul_cur_ticks = g_ul_us_ticks;
	 while ((g_ul_us_ticks - ul_cur_ticks) < ul_dly_ticks);
 }

 uint32_t micros(void){
	 return g_ul_us_ticks;
 }

 static void reverse(char *s, size_t s_len) {
	 size_t i, j;
	 char swap;

	 for (i = 0, j = s_len - 1; i < j; ++i, --j) {
		 swap = s[i];
		 s[i] = s[j];
		 s[j] = swap;
	 }
 }

 void _itoa(char *s, int32_t n) {
	 size_t i = 0;
	 long int sign_mask;
	 unsigned long int nn;

	 sign_mask = n >> sizeof(int32_t) * 8 - 1;
	 nn = (n + sign_mask) ^ sign_mask;
	 do {
		 s[i++] = nn % 10 + '0';
	 } while (nn /= 10);

	 s[i] = '-';
	 i += sign_mask & 1;
	 s[i] = '\0';

	 reverse(s, i);
 }

 double _atof(char *s){
	 
	 double val, power;
	 int i, sign;

	 for (i = 0; s[i]==' '; i++);
	 sign = (s[i] == '-') ? -1 : 1;
	 if(s[i] == '+' || s[i] == '-')
	 i++;
	 
	 for(val = 0.0; isdigit(s[i]); i++){
		 val = 10.0 * val + (s[i] - '0');
	 }
	 if(s[i] == '.')
	 i++;
	 for (power = 1.0; isdigit(s[i]); i++){
		 val = 10.0 * val + (s[i] - '0');
		 power *= 10.0;
	 }
	 return sign * val / power;
 }