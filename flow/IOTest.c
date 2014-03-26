#include "contiki.h"
#include "net/rime.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "dev/leds.h"
#include "interface/bolt_int.h"
#include "interface/photosensor_int.h"
#include "service/uart_service.h"
#include "contiki-conf.h"
#include "drivers/extflash_drv.h"

/*---------------------------------------------------------------------------*/
PROCESS(ramptest, "Test for Ramps");
AUTOSTART_PROCESSES(&ramptest);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
struct broadcast_conn broadcast;
static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
	uint8_t* msg = (uint8_t*)packetbuf_dataptr();
	if(msg[0] == UART_RAMP_RELEASE_PACKAGE){
		leds_toggle(LEDS_GREEN);
		bolt_release();
	} else if(msg[0] == UART_RAMP_SEPARATE_PACKAGE){
		leds_toggle(LEDS_RED);
		bolt_separate();
	} else if(msg[0] == UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE){
		leds_toggle(LEDS_RED | LEDS_GREEN);
		bolt_release_and_separate();
	} else if(msg[0] == UART_RAMP_NUM_PACKAGES){
		uint8_t newmsg[2];
		newmsg[0] = UART_RAMP_RET_NUM_PACKAGES;
		newmsg[1] = photosensor_num_packages();
		packetbuf_copyfrom(newmsg, 2);
		broadcast_send(&broadcast);
	} else if(msg[0] == UART_RAMP_BAY_STATUS){
		uint8_t newmsg[2];
		newmsg[0] = UART_RAMP__RET_BAY_STATUS;
		newmsg[1] = msg[1];
		newmsg[2] = photosensor_is_bay_occupied(msg[1]);
		packetbuf_copyfrom(newmsg, 2);
		broadcast_send(&broadcast);
	} else if(msg[0] == UART_RAMP_RET_NUM_PACKAGES){
		printf("Number of packages: %u\n", msg[1]);
	} else if(msg[0] == UART_RAMP__RET_BAY_STATUS){
		if(msg[2]){
			printf("There is a package in Bay %u\n", msg[1]);
		} else {
			printf("No package in Bay %u\n", msg[1]);
		}
	}
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

PROCESS_THREAD(ramptest, ev, data)
{
	static struct etimer et;
	volatile int i=0;
	
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	
	PROCESS_BEGIN();
	
	leds_off(LEDS_ALL);
	
	broadcast_open(&broadcast, 129, &broadcast_call);

	while(1) {
		leds_toggle(LEDS_YELLOW);
		etimer_set(&et, CLOCK_SECOND*4);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/