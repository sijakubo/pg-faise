/*
 * uart_service.c
 *
 * Created: 10.03.2014 10:05:14
 *  Author: JanGerd
 */ 

#include "drivers/uart_drv.h"
#include "service/uart_service.h"

void uart0_send_command(uint8_t com, uint8_t* args){
	uart0_send(&com, 1);
	int num_args = 0;
	if(com == UART_LED_GREEN_COMMAND || com == UART_LED_RED_COMMAND)
		num_args = 1;
		
	uart0_send(args, num_args);
	
	uint8_t pDelimiter = UART_COMMAND_DELEMITER;
	uart0_send(&pDelimiter, 1);
}