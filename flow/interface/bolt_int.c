/*
 * bolt_int.c
 *
 * Created: 24.02.2014 10:25:15
 *  Author: JanGerd
 */ 
#include "sys/rtimer.h"
#include "drivers/bolt_drv.h"
#include "interface/bolt_int.h"
 
void bolt_init(void){
	bolt_drv_init();
}

void bolt_release_and_separate_packet(void){
	bolts_up(BOLT_UPPER);
	
	clock_delay(BOLT_TIMEOUT);
	
	bolts_down(BOLT_LOWER);
	
	clock_delay(BOLT_SLIDE_TIME);
	
	bolts_up(BOLT_LOWER);
	
	clock_delay(BOLT_TIMEOUT);
	
	bolts_down(BOLT_UPPER);
	
	clock_delay(BOLT_SLIDE_TIME);
	
	bolts_up(BOLT_UPPER);
}