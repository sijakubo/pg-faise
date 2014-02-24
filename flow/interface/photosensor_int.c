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

unsigned int is_bay_occupied(int i){
	if(i == 1)
	return ((int)(PHOTOSENSOR_1 & get_photosensors()));
	else if(i == 2)
	return ((int)(PHOTOSENSOR_2 & get_photosensors()));
	else if(i == 3)
	return ((int)(PHOTOSENSOR_3 & get_photosensors()));
	else if(i == 4)
	return ((int)(PHOTOSENSOR_4 & get_photosensors()));
	
	return 0;
}

unsigned int is_first_bay_occupied(){
	return is_bay_occupied(1);
}

unsigned int num_packages(void){
	int i = 1, n = 0;
	for(i = 0; i<5;i++)
		if(is_bay_occupied(i))
			n++;
	return n;
}