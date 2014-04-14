/**
 * \file photosensor_int.c
 * \brief	Interface f�r die Lichtschranken
 *
 * \author	Jan-Gerd Me�
 * \date    24.02.2014
 */ 

#include "drivers/photosensor_drv.h"

/**
 * \fn	void photosensor_init(void)
 * \brief	Initialisiert das Lichtschranken-Interface:
 *
 * \author	Jan-Gerd Me�
 */
void photosensor_init(void){
	photosensor_drv_init();
}

/**
 * \fn	uint8_t photosensor_is_bay_occupied(uint8_t i)
 * \brief	Pr�ft, ob ein Platz auf der Rampe belegt ist
 *
 * \param i Platz auf der Rampe 1 <= i <= 4
 *
 * \author	Jan-Gerd Me�
 */
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

/**
 * \fn	uint8_t photosensor_is_first_bay_occupied()
 * \brief	Pr�ft, ob auf dem ersten Platz einer Rampe ein Paket liegt
 *
 * \author	Jan-Gerd Me�
 */
uint8_t photosensor_is_first_bay_occupied(){
	return photosensor_is_bay_occupied(1);
}

/**
 * \fn	uint8_t photosensor_num_packages()
 * \brief	Z�hlt die Anzahl der Pakete auf der Rampe
 *
 * \author	Jan-Gerd Me�
 */
uint8_t photosensor_num_packages(){
	uint8_t i = 1, n = 0;
	for(i = 0; i<5;i++)
		if(photosensor_is_bay_occupied(i))
			n++;
	return n;
}