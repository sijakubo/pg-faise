/*
 * radio_int.h
 *
 * Created: 27.04.2014 21:09:29
 *  Author: Malte
 */ 


#ifndef RADIO_INT_H_
#define RADIO_INT_H_

// Maximale Power des CC2420-Moduls
#define MAXPOWER 31

#include "drivers/radio_drv.h"

void radio_init(void);

#endif /* RADIO_INT_H_ */