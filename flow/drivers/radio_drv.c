/*
 * radio_drv.c
 *
 * Created: 27.04.2014 19:25:45
 *  Author: Malte
 */ 

#include "drivers/radio_drv.h"
#include "net/rime.h"

static void RadioDriver_receivemessage(struct abc_conn *c)
{
	uint8_t* msg = (uint8_t*)packetbuf_dataptr();
	
	printf("abc message receive \n");
	
	printf("msg[0] = %2x\n",msg[0]);
	printf("msg[1] = %2x\n",msg[1]);
	//Übergabe an das Agenten-RT
	//CommunicationInterface_ReceiveWirelessMessage(msg);
	
	if(msg[0] == 0x61){
		if(msg[1] == 0x01)
			leds_on(LEDS_RED);
		else
			leds_off(LEDS_RED);
	}
}

static const struct abc_callbacks abc_call = {RadioDriver_receivemessage};
static struct abc_conn abc;

void RadioDriver_sendmessage(uint8_t* msgtext, uint8_t length){
	packetbuf_copyfrom(msgtext, length);
	abc_send(&abc);
}

void RadioDriver_init(void){
	cc2420_set_txpower(MAXPOWER);
	abc_open(&abc, 128, &abc_call);
}