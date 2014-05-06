/*
 * radio_int.c
 *
 * Created: 27.04.2014 21:08:13
 *  Author: Malte
 */ 

#include "interface/radio_int.h"

void radio_init(void){
	cc2420_set_txpower(MAXPOWER);
	openabcchannel();
}