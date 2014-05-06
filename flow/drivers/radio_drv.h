/*
 * radio_drv.h
 *
 * Created: 27.04.2014 19:26:00
 *  Author: Malte
 */ 


#ifndef RADIO_DRV_H_
#define RADIO_DRV_H_

#include "contiki.h"
#include "net/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>

// Maximale Power des C4220-Moduls
#define MAXPOWER 31
// Ausschalten des C4220-Moduls
#define POWEROUT 0

void RadioDriver_sendmessage(uint8_t* msgtext, uint8_t length);
static void RadioDriver_receivemessage(struct abc_conn *c);
void RadioDriver_init(void);

#endif /* RADIO_DRV_H_ */