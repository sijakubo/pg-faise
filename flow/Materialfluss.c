/*
 * com_service.c
 *
 * Created: 05.04.2014 12:37:14
 *  Author: Malte Falk und Jan-Gerd Meﬂ
 */ 

#define COM_CMD_RECEIVE_PACKAGE 0x50
#define COM_CMD_DELIVER_PACKAGE 0x55
#define COM_CMD_RET_COST 0x11

#define COM_CMD_DRIVE_IN_RAMP 0x01
#define COM_CMD_DRIVE_OUT_RAMP 0x02
#define COM_CMD_COST 0x10
#define COM_CMD_RECEIVE_PACKAGE_ACK 0x20
#define COM_CMD_DELIVER_PACKAGE_ACK 0x25

//*********************************************
// Test-Commands for Ramps
//*********************************************
#define UART_LED_RED_COMMAND 0x61
#define UART_LED_GREEN_COMMAND 0x62
#define UART_RAMP_NUM_PACKAGES 0x63
#define UART_RAMP_RET_NUM_PACKAGES 0x64
#define UART_RAMP_BAY_STATUS 0x65
#define UART_RAMP_RET_BAY_STATUS 0x66
#define UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE 0x67
#define UART_RAMP_SEPARATE_PACKAGE 0x68
#define UART_RAMP_RELEASE_PACKAGE 0x69
//*********************************************

#include "service/com_service.h"

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
		if(msg[1]){
			RadioDriver_sendmessage(msg,2);
			leds_on(LEDS_GREEN);
		}
		else
		leds_off(LEDS_GREEN);
	}
	else if(msg[0] == UART_LED_RED_COMMAND){
		if(msg[1]){
			leds_on(LEDS_RED);
			RadioDriver_sendmessage(msg,2);
		}
		else{
			leds_off(LEDS_RED);
			RadioDriver_sendmessage(msg,2);
		}
	} else if(msg[0] == UART_RAMP_NUM_PACKAGES){
		printf("Asking for number of packages...\n");
	} else if(msg[0] == UART_RAMP_BAY_STATUS){
		printf("Asking for Status of Bay %u...\n", msg[1]);
	} else if(msg[0] == UART_RAMP_SEPARATE_PACKAGE){
		printf("Separating Package...\n");
	} else if(msg[0] == UART_RAMP_RELEASE_PACKAGE){
		printf("Release Package...\n");
	}  else if(msg[0] == UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE){
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
