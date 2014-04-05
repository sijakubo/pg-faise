/*
 * com_service.c
 *
 * Created: 05.04.2014 12:37:14
 *  Author: Malte Falk und Jan-Gerd Meﬂ
 */ 

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
}

void com_send(unit8_t* msg, uint8_t len){
	if ((msg[0] | 0x7f) == 0x7f)
	{
		uart0_send(msg, len);
	} else {
		// To-Do: WIRELESS COMMUNIACTION
	}
}
