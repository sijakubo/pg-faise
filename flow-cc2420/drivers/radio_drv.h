/*
 * wirelesscomm.h
 *
 * Created: 27.04.2014 19:26:00
 *  Author: Malte
 */ 


#ifndef WIRELESSCOMM_H_
#define WIRELESSCOMM_H_

//#include "dev/cc2420.h"

// Maximale Power des C4220-Moduls
#define MAXPOWER 31
// Ausschalten des C4220-Moduls
#define POWEROUT 0

void radio_init_drv(void);

#endif /* WIRELESSCOMM_H_ */