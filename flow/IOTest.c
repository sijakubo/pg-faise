#include "contiki.h"
#include "net/rime.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "dev/leds.h"
#include "interface/bolt_int.h"
#include "interface/photosensor_int.h"
#include "service/com_service.h"
#include "contiki-conf.h"
#include "drivers/extflash_drv.h"

/*---------------------------------------------------------------------------*/
PROCESS(ramptest, "Test for Ramps");
AUTOSTART_PROCESSES(&ramptest);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
struct abc_conn broadcast;
static void broadcast_recv(struct abc_conn *c)
{
	uint8_t* msg = (uint8_t*)packetbuf_dataptr();
	if(msg[0] == UART_RAMP_RELEASE_PACKAGE){
		leds_toggle(LEDS_GREEN);
		BoltInterface_release();
	} else if(msg[0] == UART_RAMP_SEPARATE_PACKAGE){
		leds_toggle(LEDS_RED);
		BoltInterface_separate();
	} else if(msg[0] == UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE){
		leds_toggle(LEDS_RED | LEDS_GREEN);
		BoltInterface_release_and_separate();
	} else if(msg[0] == UART_RAMP_NUM_PACKAGES){
		uint8_t newmsg[2];
		newmsg[0] = UART_RAMP_RET_NUM_PACKAGES;
		newmsg[1] = PhotosensorInterface_num_packages();
		packetbuf_copyfrom(newmsg, 2);
		abc_send(&broadcast);
	} else if(msg[0] == UART_RAMP_BAY_STATUS){
		uint8_t newmsg[3];
		newmsg[0] = UART_RAMP_RET_BAY_STATUS;
		newmsg[1] = msg[1];
		newmsg[2] = PhotosensorInterface_is_bay_occupied(msg[1]);
		packetbuf_copyfrom(newmsg, 2);
		abc_send(&broadcast);
	} else if(msg[0] == UART_RAMP_RET_NUM_PACKAGES){
		printf("Number of packages: %u\n", msg[1]);
	} else if(msg[0] == UART_RAMP_RET_BAY_STATUS){
		if(msg[2]){
			printf("There is a package in Bay %u\n", msg[1]);
		} else {
			printf("No package in Bay %u\n", msg[1]);
		}
	}
}

static const struct abc_callbacks broadcast_call = {broadcast_recv};

PROCESS_THREAD(ramptest, ev, data)
{
	static struct etimer et;
	
	PROCESS_EXITHANDLER(abc_close(&broadcast);)
	
	PROCESS_BEGIN();
	
	leds_off(LEDS_ALL);
	
	abc_open(&broadcast, 129, &broadcast_call);

	while(1) {
		leds_toggle(LEDS_YELLOW);
		etimer_set(&et, CLOCK_SECOND*4);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/