/*
 * photosensor_drv.c
 *
 * Created: 24.02.2014 16:43:11
 *  Author: JanGerd
 */ 
#include "drivers/photosensor_drv.h"

void photosensor_drv_init(void){
	PHOTOSENSORS_PxDIR &= 0x0F;
	PHOTOSENSORS_PxOUT |= 0xF0;
}

uint8_t get_photosensors(){
	uint8_t res = 0xF0 & PHOTOSENSORS_PxIN;
	return res>>4;
}