/*
 * bolt.h
 *
 * Created: 24.02.2014 10:00:04
 *  Author: JanGerd
 */ 


#ifndef BOLT_DRV_H_
#define BOLT_DRV_H_

#include "contiki-conf.h"

#define BOLT_1 1
#define BOLT_2 2
#define BOLTS_PxDIR DDRC
#define BOLTS_PxOUT PORTC

void bolt_drv_init(void);
void bolts_up(unsigned char boltv);
void bolts_down(unsigned char boltv);
void bolts_toggle(unsigned char boltv);
uint8_t get_bolts(void);

#endif