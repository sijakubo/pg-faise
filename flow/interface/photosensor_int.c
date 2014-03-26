/*
 * photosensor_int.c
 *
 * Created: 24.02.2014 16:43:37
 *  Author: JanGerd
 */ 
#include "drivers/photosensor_drv.h"

void photosensor_init(void){
	photosensor_drv_init();
}

uint8_t photosensor_is_bay_occupied(uint8_t i){
	if(i == 1)
		return ((uint8_t)((1<<PHOTOSENSOR_1) & get_photosensors()));
	else if(i == 2)
		return ((uint8_t)((1<<PHOTOSENSOR_2) & get_photosensors()));
	else if(i == 3)
		return ((uint8_t)((1<<PHOTOSENSOR_3) & get_photosensors()));
	else if(i == 4)
		return ((uint8_t)((1<<PHOTOSENSOR_4) & get_photosensors()));
	
	return 0;
}

uint8_t photosensor_is_first_bay_occupied(){
	return photosensor_is_bay_occupied(1);
}

uint8_t photosensor_num_packages(void){
	uint8_t i = 1, n = 0;
	for(i = 0; i<5;i++)
		if(photosensor_is_bay_occupied(i))
			n++;
	return n;
}