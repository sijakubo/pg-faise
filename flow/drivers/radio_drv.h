/*
 * radio_drv.h
 *
 * Created: 27.04.2014 19:26:00
 *  Author: Malte
 */ 


#ifndef RADIO_DRV_H_
#define RADIO_DRV_H_

//#include "dev/cc2420.h"

// Maximale Power des C4220-Moduls
#define MAXPOWER 31
// Ausschalten des C4220-Moduls
#define POWEROUT 0

void radio_init_drv(void);

#endif /* RADIO_DRV_H_ */