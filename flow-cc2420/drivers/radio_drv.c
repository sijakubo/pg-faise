/*
 * wirelesscomm.c
 *
 * Created: 27.04.2014 19:25:45
 *  Author: Malte
 */ 

#include "drivers/radio_drv.h"

//#include "contiki.h"
//#include "net/rime.h"
//#include "contiki-conf.h"


void radio_init_drv(void){
	cc2420_set_txpower(POWEROUT);
}
