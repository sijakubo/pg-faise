/*
 * photosensor_drv.h
 *
 * Created: 24.02.2014 16:43:24
 *  Author: JanGerd
 */ 


#ifndef PHOTOSENSOR_DRV_H_
#define PHOTOSENSOR_DRV_H_

#include "contiki-conf.h"

#define PHOTOSENSORS_PxDIR DDRC
#define PHOTOSENSORS_PxOUT PORTC
#define PHOTOSENSORS_PxIN PINC;

#define PHOTOSENSOR_1 1<<PC4
#define PHOTOSENSOR_2 1<<PC5
#define PHOTOSENSOR_3 1<<PC6
#define PHOTOSENSOR_4 1<<PC7

void photosensor_drv_init(void);
uint8_t get_photosensors();



#endif /* PHOTOSENSOR_DRV_H_ */