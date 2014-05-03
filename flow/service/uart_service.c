/*
 * uart_service.c
 *
 * Created: 10.03.2014 10:05:14
 *  Author: JanGerd
 */ 

#include "drivers/uart_drv.h"
#include "service/uart_service.h"
#include "net/rime.h"
#include "leds.h"

extern struct broadcast_conn broadcast;

void uart0_send_command(uint8_t com, uint8_t* args){
	int num_args = 0;
	
	if(com == UART_LED_GREEN_COMMAND || com == UART_LED_RED_COMMAND)
		num_args = UART_LED_COMMAND_ARGS;
	
	uint8_t pDelimiter = UART0_LINE_END;
	
	uart0_send(&com, 1);
	uart0_send(&pDelimiter, 1);
	uart0_send(args, num_args);
}

void uart0_dispatch_message(uint8_t* msg){
	if(msg[0] == UART_LED_GREEN_COMMAND){
		if(msg[1])
			leds_on(LEDS_GREEN);
		else
			leds_off(LEDS_GREEN);
	}
	else if(msg[0] == UART_LED_RED_COMMAND){
		if(msg[1])
			leds_on(LEDS_RED);
		else
			leds_off(LEDS_RED);
	} else if(msg[0] == UART_RAMP_NUM_PACKAGES){
		packetbuf_copyfrom(msg, 1);
		broadcast_send(&broadcast);
		printf("Asking for number of packages...\n");
	} else if(msg[0] == UART_RAMP_SEPARATE_PACKAGE){
		packetbuf_copyfrom(msg, 1);
		broadcast_send(&broadcast);
		printf("Separating Package...\n");
	} else if(msg[0] == UART_RAMP_RELEASE_PACKAGE){
		packetbuf_copyfrom(msg, 1);
		broadcast_send(&broadcast);
		printf("Release Package...\n");
	} else if(msg[0] == UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE){
		packetbuf_copyfrom(msg, 1);
		broadcast_send(&broadcast);
		printf("Release and Separate Package...\n");
	}
}