/*
 * pcbtop_stabilizer.c
 *
 * Created: 13/11/2016 20:06:28
 *  Author: Badr
 */ 

 #include "pcbtop_stabilizer.h"

 #include "ctype.h"

 #ifdef __cplusplus
 extern "C" {
#endif





//-- GPIO
	//ioport_enable_pin(PIO_PA0_IDX);
	//ioport_set_pin_dir(PIO_PA0_IDX,IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(PIO_PA0_IDX,false);
	//
	//ioport_enable_pin(PIO_PB5_IDX);
	//ioport_set_pin_dir(PIO_PB5_IDX,IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(PIO_PB5_IDX,true);
	//
	//ioport_enable_pin(PIO_PB4_IDX);
	//ioport_set_pin_dir(PIO_PB4_IDX,IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(PIO_PB4_IDX,false);
	//
	//ioport_enable_pin(PIO_PD11_IDX);
	//ioport_set_pin_dir(PIO_PD11_IDX,IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(PIO_PD11_IDX,true);



 //-- PWM
 // TINKER_PWM_1 -> PWM0 -> PA0 -> PWMC0_PWMH0
 // TINKER_PWM_2 -> PWM0 -> PA2 -> PWMC0_PWMH1
 // TINKER_PWM_3 -> PWM0 -> PA13 -> PWMC0_PWMH2
 // TINKER_PWM_4 -> PWM0 -> PA7 -> PWMC0_PWMH7

 void tinkerPwmInit(uint32_t pwmId){
	
	Pwm* pwm = NULL;
	uint32_t id_pwm = 0;
	uint32_t pwm_channel = 0;

	switch (pwmId) {
		case TINKER_PWM_1:
			pwm = PWM0;
			id_pwm = ID_PWM0;
			pwm_channel = PWM_CHANNEL_0;
			pwm_channel_instance.ul_prescaler = PWM_CMR_CPRE_CLKA;
			pwm_channel_instance.b_pwmh_output_inverted = false;
			ioport_set_pin_mode(PIO_PA0_IDX, IOPORT_MODE_MUX_A);
			ioport_disable_pin(PIO_PA0_IDX);
			break;
		case TINKER_PWM_2:
			pwm = PWM0;
			id_pwm = ID_PWM0;
			pwm_channel = PWM_CHANNEL_1;
			pwm_channel_instance.ul_prescaler = PWM_CMR_CPRE_CLKA;
			pwm_channel_instance.b_pwmh_output_inverted = true;
			ioport_set_pin_mode(PIO_PA2_IDX, IOPORT_MODE_MUX_A);
			ioport_disable_pin(PIO_PA2_IDX);
			break;
		case TINKER_PWM_3:
			pwm = PWM0;
			id_pwm = ID_PWM0;
			pwm_channel = PWM_CHANNEL_2;
			pwm_channel_instance.ul_prescaler = PWM_CMR_CPRE_CLKB;
			pwm_channel_instance.b_pwmh_output_inverted = true;
			ioport_set_pin_mode(PIO_PA13_IDX, IOPORT_MODE_MUX_B);
			ioport_disable_pin(PIO_PA13_IDX);
			break;
		case TINKER_PWM_4:
			pwm = PWM0;
			id_pwm = ID_PWM0;
			pwm_channel = PWM_CHANNEL_3;
			pwm_channel_instance.ul_prescaler = PWM_CMR_CPRE_CLKB;
			pwm_channel_instance.b_pwmh_output_inverted = true;
			ioport_set_pin_mode(PIO_PA7_IDX, IOPORT_MODE_MUX_B);
			ioport_disable_pin(PIO_PA7_IDX);
			break;
		default:
			return;
	}
	pmc_enable_periph_clk(id_pwm);
	pwm_channel_disable(pwm, pwm_channel);

	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(pwm, &clock_setting);

	//--
	
	pwm_channel_instance.ul_period = PERIOD_VALUE;
	pwm_channel_instance.ul_duty = 0;
	pwm_channel_instance.channel = pwm_channel;
	pwm_channel_instance.alignment = PWM_ALIGN_LEFT;
	
	
	pwm_channel_init(pwm, &pwm_channel_instance);

	//--
	pwm_channel_enable(pwm, pwm_channel);

 }

 void tinkerPwmWrite(uint32_t pwmId, uint32_t duty){
	Pwm* pwm = NULL;
	uint32_t id_pwm = 0;
	uint32_t pwm_channel = 0;

	 switch (pwmId) {
		 case TINKER_PWM_1:
			 pwm = PWM0;
			 id_pwm = ID_PWM0;
			 pwm_channel = PWM_CHANNEL_0;
			 break;
		 case TINKER_PWM_2:
			 pwm = PWM0; //PWM0 ok
			 id_pwm = ID_PWM0;
			 pwm_channel = PWM_CHANNEL_1;
			 break;
		 case TINKER_PWM_3:
			 pwm = PWM0;
			 id_pwm = ID_PWM0;
			 pwm_channel = PWM_CHANNEL_2;
			 break;
		 case TINKER_PWM_4:
			 pwm = PWM0;
			 id_pwm = ID_PWM0;
			 pwm_channel = PWM_CHANNEL_3;
			 break;
		 default:
			return;
	 }

	 pwm_channel_disable(pwm, pwm_channel);

	 //--
	 pwm_channel_instance.ul_duty = duty;
	 pwm_channel_instance.channel = pwm_channel;	 
	 pwm_channel_init(pwm, &pwm_channel_instance);

	
	 //--
	 pwm_channel_enable(pwm, pwm_channel);

 }



 //----------------------------------------------------
 //-- UART
 //----------------------------------------------------

 void tinkerUartInit(uint32_t uartId){
	//ioport_set_pin_mode(PIO_PA10_IDX, IOPORT_MODE_MUX_A);
	//ioport_disable_pin(PIO_PA10_IDX);
	//ioport_set_pin_mode(PIO_PA9_IDX, IOPORT_MODE_MUX_A);'
	//ioport_disable_pin(PIO_PA9_IDX);

	sam_uart_opt_t usart_settings;
	usart_settings.ul_mck = sysclk_get_peripheral_hz(); //sysclk_get_cpu_hz();//
	usart_settings.ul_baudrate = TINKER_UART_BAUDE;
	usart_settings.ul_mode = 0	| UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;//  UART_MR_BRSRCCK_PERIPH_CLK ;

	switch (uartId) {
		case TINKER_UART_0:
			pmc_enable_periph_clk(ID_UART0);
			pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA10);
			pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA9);
			uart_init((Uart*)UART0, &usart_settings);
			break;
		case TINKER_UART_1:
			return;//TBD
			break;
		case TINKER_UART_2:
			return;//TBD
			break;
		case TINKER_UART_3:
			return;//TBD
			break;
		case TINKER_UART_4:
			return;//TBD
			break;
		default:
		return;
	}

	return; 
//
//
	//pmc_enable_periph_clk(ID_UART0);
	////pmc_enable_periph_clk(ID_PIOA);
//
	//pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA10);
	//pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA9);
//
//
	////-- 
	////sysclk_enable_peripheral_clock(ID_UART0);
	//
	////NVIC_DisableIRQ(UART0_IRQn);
	//
	////--
	//sam_uart_opt_t usart_settings;
	//usart_settings.ul_mck = sysclk_get_peripheral_hz(); //sysclk_get_cpu_hz();//
	//usart_settings.ul_baudrate = TINKER_UART_BAUDE;
	//usart_settings.ul_mode = 0	| UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;//  UART_MR_BRSRCCK_PERIPH_CLK ;
	//uart_init((Uart*)UART0, &usart_settings);
	////stdio_serial_init((Uart*)UART0, &usart_settings);
	//
	////--
	////uart_enable_rx(UART0);
	////uart_enable_tx(UART0);
 }

 void tinkerUartGetChar(uint32_t uartId, uint8_t* data){
	switch (uartId) {
		case TINKER_UART_0:
			return usart_serial_getchar((Uart*)UART0,data);
			break;
		case TINKER_UART_1:
			return usart_serial_getchar((Uart*)UART1,data);
			break;
		case TINKER_UART_2:
			return usart_serial_getchar((Uart*)UART2,data);
			break;
		case TINKER_UART_3:
			return usart_serial_getchar((Uart*)UART3,data);
			break;
		case TINKER_UART_4:
			return usart_serial_getchar((Uart*)UART4,data);
			break;
		default:
			return;
	}
 }

 void tinkerUartPutChar(uint32_t uartId, const uint8_t c){
	switch (uartId) {
		case TINKER_UART_0:
			usart_serial_putchar((Uart*)UART0, c);
			break;
		case TINKER_UART_1:
			usart_serial_putchar((Uart*)UART1, c);
			break;
		case TINKER_UART_2:
			usart_serial_putchar((Uart*)UART2, c);
			break;
		case TINKER_UART_3:
			usart_serial_putchar((Uart*)UART3, c);
			break;
		case TINKER_UART_4:
			usart_serial_putchar((Uart*)UART4, c);
			break;
		default:
		return;
	}
	
 }

 void tinkerUartPutBuffer(uint32_t uartId, const uint8_t *c, size_t size){
	while (size--) {
		tinkerUartPutChar(uartId,*c++);
	}
	//for(int i=0; i<size; i++){
		//usart_serial_putchar((Uart*)UART0, c[i]);
	//}
 }

  void tinkerUartPutString(uint32_t uartId, const char *str){
	  if (str == NULL) return;
	  return tinkerUartPutBuffer(uartId, (const uint8_t *)str, strlen(str));
  }
//
 //size_t write(const char *str) {
	 //if (str == NULL) return 0;
	 //return write((const uint8_t *)str, strlen(str));
 //}
 //size_t write(const char *buffer, size_t size) {
	 //return write((const uint8_t *)buffer, size);
 //}
//
 //size_t write(const uint8_t *buffer, size_t size)
 //{
	 //size_t n = 0;
	 //while (size--) {
		 //n += write(*buffer++);
	 //}
	 //return n;
 //}


 //-----------------------------------
 //-- I2C
 //-----------------------------------

 static twihs_package_t twihs1_packet_read;
 static twihs_package_t twihs1_packet_write;

 uint32_t tinkerInitI2C(uint8_t chip)
 {	
	
	//pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PB5);
	//pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PB4);

	 gpio_configure_pin(PIO_PB4_IDX, PIO_PERIPH_A | PIO_DEFAULT);   // TWI1 data
	 gpio_configure_pin(PIO_PB5_IDX, PIO_PERIPH_A | PIO_DEFAULT);   // TWI1 clock

	 pmc_enable_periph_clk(ID_TWIHS1);

	 // disable JTAG
	 MATRIX->CCFG_SYSIO |= (1 << 4) | (1 << 5);

	 twihs_master_options_t opt = {
		 .speed = 400000,
		 .chip  = chip //not used...
	 };

	 twihs1_packet_read.chip = chip;
	 twihs1_packet_read.addr_length  = 1;    // TWIHS slave memory address data size
	 twihs1_packet_read.length       = 1 ;                   // transfer data size (bytes)

	 twihs1_packet_write.chip = chip;
	 twihs1_packet_write.addr_length  = 1;    // TWIHS slave memory address data size
	 twihs1_packet_write.length       = 1 ;                   // transfer data size (bytes)

	 return twihs_master_setup(TWIHS1, &opt);
 }

uint8_t tinkerReadI2C(uint8_t chip, uint8_t reg){
	uint8_t val = 0;
	twihs1_packet_read.addr[0]         = reg;
	
	twihs1_packet_read.chip         = chip;     // TWIHS slave bus address
	twihs1_packet_read.buffer       = &val;        // transfer data destination buffer
	
	twihs_master_read(TWIHS1, &twihs1_packet_read);
	return val;
}

//uint16_t tinkerReadI2C_16(uint8_t reg1,uint8_t reg2){
	//uint16_t val = 0;
	//twihs1_packet_read.addr[0]         = reg1;
	//twihs1_packet_read.addr[1]         = reg2;
	//twihs1_packet_read.addr_length  = 2;    // TWIHS slave memory address data size
	////twihs1_packet_write.chip         = 0x68;     // TWIHS slave bus address
	//twihs1_packet_read.buffer       = &val;        // transfer data destination buffer
	//twihs1_packet_read.length       = 2 ;                   // transfer data size (bytes)
	//twihs_master_read(TWIHS1, &twihs1_packet_read);
//}

uint32_t tinkerWriteI2C(uint8_t chip, uint8_t reg, uint8_t val){
	twihs1_packet_write.addr[0]         = reg;
	
	twihs1_packet_write.chip         = chip;     // TWIHS slave bus address
	twihs1_packet_write.buffer       = &val;        // transfer data destination buffer
	
	return twihs_master_write(TWIHS1, &twihs1_packet_write);
}


 #ifdef __cplusplus
 }
 #endif