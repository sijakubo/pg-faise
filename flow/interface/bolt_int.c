/*
 * bolt_int.c
 *
 * Created: 24.02.2014 10:25:15
 *  Author: JanGerd
 */ 
#include "sys/rtimer.h"
#include "drivers/bolt_drv.h"
#include "interface/bolt_int.h"

PROCESS(bolt_int_release, "Separate Package"); 
PROCESS(bolt_int_release_and_separate, "Release and Separate Package");
PROCESS(bolt_int_separate, "Separate Package");

static struct etimer bolt_timer;
static uint8_t bolt_state = 0;

PROCESS_THREAD(bolt_int_release, ev, data)
{
	bolt_state = 0;
	uint16_t timeout = 0;
	
	PROCESS_BEGIN();
	while(bolt_state < 2){
		if(bolt_state == 0){
			bolts_up(BOLT_UPPER);
			timeout = BOLT_TIMEOUT;
		} else if(bolt_state == 1) {
			bolts_down(BOLT_LOWER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		bolt_state++;
	}
	bolts_up(BOLT_LOWER);
	PROCESS_END();
}

PROCESS_THREAD(bolt_int_release_and_separate, ev, data)
{
	bolt_state = 0;
	uint16_t timeout = 0;
	
	PROCESS_BEGIN();
		while(bolt_state < 4){
			if(bolt_state == 0){
				bolts_up(BOLT_UPPER);
				timeout = BOLT_TIMEOUT;
			} else if(bolt_state == 1) {
				bolts_down(BOLT_LOWER);
				timeout = BOLT_SLIDE_TIME;
			} else if(bolt_state == 2) {
				bolts_up(BOLT_LOWER);
				timeout = BOLT_TIMEOUT;
			} else if(bolt_state == 3) {
				bolts_down(BOLT_UPPER);
				timeout = BOLT_SLIDE_TIME;
			}
			etimer_set(&bolt_timer, timeout);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
			bolt_state++;
		}
		bolts_up(BOLT_UPPER);
	PROCESS_END();
}

PROCESS_THREAD(bolt_int_separate, ev, data)
{
	bolt_state = 0;
	uint16_t timeout = 0;
	
	PROCESS_BEGIN();
	while(bolt_state < 2){
		if(bolt_state == 0){
			bolts_up(BOLT_LOWER);
			timeout = BOLT_TIMEOUT;
		} else if(bolt_state == 1) {
			bolts_down(BOLT_UPPER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		bolt_state++;
	}
	bolts_up(BOLT_UPPER);
	PROCESS_END();
}

void bolt_release(){
	process_start(&bolt_int_release, NULL);
}

void bolt_release_and_separate(){
	process_start(&bolt_int_release_and_separate, NULL);
}

void bolt_separate(){
	process_start(&bolt_int_separate, NULL);
}

void bolt_init(void){
	bolt_drv_init();
}