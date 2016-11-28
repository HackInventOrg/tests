/*
 * TinkerUart.h
 *
 * Created: 15/11/2016 23:43:56
 *  Author: Badr
 */ 


#ifndef TINKERUART_H_
#define TINKERUART_H_

#include "conf_board.h"


double _atof(char *s);
void _itoa(char*,int32_t);

typedef class TinkerUart {
	
	private:
		uint32_t _uartId = TINKER_UART_0;
		char _char_number[20];
	public:
		TinkerUart(uint32_t uartId){_uartId = uartId;};

		void init(){
			tinkerUartInit(_uartId);
		};
		void read(uint8_t *str){
			tinkerUartGetChar(_uartId,str);
		};
		void writeDouble(double val){
			char * ch =(char*) &val;
			tinkerUartPutChar(_uartId,ch[0]);
			tinkerUartPutChar(_uartId,ch[1]);
			tinkerUartPutChar(_uartId,ch[2]);
			tinkerUartPutChar(_uartId,ch[3]);
			tinkerUartPutChar(_uartId,ch[4]);
			tinkerUartPutChar(_uartId,ch[5]);
			tinkerUartPutChar(_uartId,ch[6]);
			tinkerUartPutChar(_uartId,ch[7]);
		}
		void write16u(uint16_t val){
			
			tinkerUartPutChar(_uartId,val>>8);
			tinkerUartPutChar(_uartId,val&0b0000000011111111);
			
		}
		void write(char c){
			tinkerUartPutChar(_uartId, c);
		}
		void writeln(char c){
			tinkerUartPutChar(_uartId, c);
			print("\n");
		}
		void print(const char *str){
			tinkerUartPutString(_uartId,str);
		};
		void println(const char *str){
			tinkerUartPutString(_uartId,str);
			tinkerUartPutString(_uartId,"\n");
		};
		void printNumber(uint32_t number){
			_itoa(_char_number,number);
			print(_char_number);
		}
		void printNumberCaca(uint32_t number){
			_itoa(_char_number,number);
			print(_char_number);
		}
		void printlnNumber(uint32_t number){
			_itoa(_char_number,number);
			println(_char_number);
		}

		void printlnDouble(double number){
			printDouble(number, 10);
			print("\n");
		}

		void printDouble(double number){
			printDouble(number, 10);
		}

		void printDouble(double number, uint8_t digits)
		{
			
			size_t n = 0;
			
			//if (isnan(number)) return print("nan");
			//if (isinf(number)) return print("inf");
			if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
			if (number <-4294967040.0) return print ("ovf");  // constant determined empirically
			
			// Handle negative numbers
			if (number < 0.0)
			{
				//n += print('-');
				print("-");
				number = -number;
			}

			

			// Round correctly so that print(1.999, 2) prints as "2.00"
			double rounding = 0.5;
			for (uint8_t i=0; i<digits; ++i)
			rounding /= 10.0;
			
			number += rounding;

			// Extract the integer part of the number and print it
			unsigned long int_part = (unsigned long)number;
			double remainder = number - (double)int_part;
			//n += print(int_part);
			printNumber(int_part);

			// Print the decimal point, but only if there are digits beyond
			if (digits > 0) {
				//n += print(".");
				print(".");
			}
			
			// Extract digits from the remainder one at a time
			while( digits-- > 0 )
			{
				remainder *= 10.0;
				int toPrint = int( remainder );
				printNumber( toPrint );
				remainder -= toPrint;
			}
			
			

			return;
			
			//return n;
		}
		
 }t_TinkerUart;

 static  TinkerUart TinkerUart0(TINKER_UART_0);
 //static  TinkerUart TinkerUart1(TINKER_UART_1); ----> pins not defined yet
 //static  TinkerUart TinkerUart2(TINKER_UART_2);
 //static  TinkerUart TinkerUart3(TINKER_UART_3);
 //static  TinkerUart TinkerUart4(TINKER_UART_4);

#endif /* TINKERUART_H_ */