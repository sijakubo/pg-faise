/*
 * com_service.c
 *
 * Created: 05.04.2014 12:37:14
 *  Author: Malte Falk und Jan-Gerd Meﬂ
 */ 

#include "service/com_service.h"

extern struct broadcast_conn broadcast;

void com_receive(uint8_t* msg){
	if (msg[0]==COM_CMD_RECEIVE_PACKAGE)
	{
		printf("%u: Package %u%u arrived at Ramp %u%u", node_id, msg[1], msg[2], msg[3], msg[4]);
	}
	else if (msg[0]==COM_CMD_DELIVER_PACKAGE)
	{
		
	}
	else if (msg[0]==COM_CMD_RET_COST)
	{
	}
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
	} else if(msg[0] == UART_RAMP_BAY_STATUS){
		packetbuf_copyfrom(msg, 2);
		broadcast_send(&broadcast);
		printf("Asking for Status of Bay %u...\n", msg[1]);
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

void com_send(uint8_t* msg, uint8_t len){
	if ((msg[0] | 0x7f) == 0x7f)
	{
		uart0_send(msg, len);
	} else {
		// To-Do: WIRELESS COMMUNIACTION
	}
}
