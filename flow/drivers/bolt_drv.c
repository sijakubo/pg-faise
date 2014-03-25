/*
 * bolt.c
 *
 * Created: 24.02.2014 09:59:36
 *  Author: JanGerd
 */ 
#include "drivers/bolt_drv.h"

void bolt_drv_init(void){
	BOLTS_PxDIR |= (BOLT_1 | BOLT_2);
	BOLTS_PxOUT |= (BOLT_1 | BOLT_2);
}

void bolts_up(unsigned char boltv){
	boltv &= 0x03;
	BOLTS_PxOUT |= boltv;
}

void bolts_down(unsigned char boltv){
	boltv &= 0x03;
	BOLTS_PxOUT &= ~boltv; 
}

void bolts_toggle(unsigned char boltv){
	boltv &= 0x03;
	if(boltv & BOLT_1){
		BOLTS_PxOUT ^= BOLT_1;
	}
	if(boltv & BOLT_2){
		BOLTS_PxOUT ^= BOLT_2;
	}
}

uint8_t get_bolts(void){
	return 0x0F & BOLTS_PxOUT;
}