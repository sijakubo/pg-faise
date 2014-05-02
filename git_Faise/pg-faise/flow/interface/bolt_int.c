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

PROCESS_THREAD(bolt_int_release, ev, data)
{
	static uint8_t state = 0;
	static struct etimer bolt_timer;
	uint16_t timeout = 0;
	PROCESS_BEGIN();
	while(state < 2){
		if(state == 0){
			bolts_up(BOLT_UPPER);
			timeout = BOLT_TIMEOUT;
		} else if(state == 1) {
			bolts_down(BOLT_LOWER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		state++;
	}
	bolts_up(BOLT_LOWER);
	PROCESS_END();
}

PROCESS_THREAD(bolt_int_release_and_separate, ev, data)
{
	static uint8_t state = 0;
	static struct etimer bolt_timer;
	uint16_t timeout = 0;
	PROCESS_BEGIN();
		while(state < 4){
			if(state == 0){
				bolts_up(BOLT_UPPER);
				timeout = BOLT_TIMEOUT;
			} else if(state == 1) {
				bolts_down(BOLT_LOWER);
				timeout = BOLT_SLIDE_TIME;
			} else if(state == 2) {
				bolts_up(BOLT_LOWER);
				timeout = BOLT_TIMEOUT;
			} else if(state == 3) {
				bolts_down(BOLT_UPPER);
				timeout = BOLT_SLIDE_TIME;
			}
			etimer_set(&bolt_timer, timeout);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
			state++;
		}
		bolts_up(BOLT_UPPER);
	PROCESS_END();
}

PROCESS_THREAD(bolt_int_separate, ev, data)
{
	static uint8_t state = 0;
	static struct etimer bolt_timer;
	uint16_t timeout = 0;
	PROCESS_BEGIN();
	while(state < 2){
		if(state == 0){
			bolts_up(BOLT_LOWER);
			timeout = BOLT_TIMEOUT;
		} else if(state == 1) {
			bolts_down(BOLT_UPPER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		state++;
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