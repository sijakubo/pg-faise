/*
 * radio_service.c
 *
 * Created: 28.04.2014 10:25:31
 *  Author: Malte
 */ 

#include "service/radio_service.h"

static void abc_recv(struct abc_conn *c, const rimeaddr_t *from)
{
	uint8_t* msg = (uint8_t*)packetbuf_dataptr();
	printf("abc message 1 received '%x'\n", msg[0]);
	printf("abc message 2 received '%x'\n", msg[1]);
/*
 * TO-DO: Empfangen und separieren der Nachricht implementieren
 */

/*
 * TO-DO: Weiter geben der empfangen Nachricht an das Agenten-RT
 */
}

static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;

void openabcchannel(void){
	abc_open(&abc, 128, &abc_call);
}

void sendmessage(uint8_t* msgtext, uint8_t length){
	
	printf("message 1 in radio_service: %x\n",msgtext[0]);
	printf("message 1 in radio_service: %x\n",msgtext[1]);
	packetbuf_copyfrom(msgtext, length);
	abc_send(&abc);
}