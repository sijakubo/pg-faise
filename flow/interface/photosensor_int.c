/**
 * \file photosensor_int.c
 * \brief	Interface für die Lichtschranken
 *
 * \author	Jan-Gerd Meß
 * \date    24.02.2014
 */ 

#include "drivers/photosensor_drv.h"

/**
 * \fn	void PhotosensorInterface_init(void)
 * \brief	Initialisiert das Lichtschranken-Interface:
 *
 * \author	Jan-Gerd Meß
 */
void PhotosensorInterface_init(void){
	PhotosensorDriver_init();
}

/**
 * \fn	uint8_t PhotosensorInterface_is_bay_occupied(uint8_t i)
 * \brief	Prüft, ob ein Platz auf der Rampe belegt ist
 *
 * \param i Platz auf der Rampe 1 <= i <= 4
 *
 * \author	Jan-Gerd Meß
 */
uint8_t PhotosensorInterface_is_bay_occupied(uint8_t i){
	if(i == 1)
		return ((uint8_t)((1<<PHOTOSENSOR_1) & PhotosensorDriver_get()));
	else if(i == 2)
		return ((uint8_t)((1<<PHOTOSENSOR_2) & PhotosensorDriver_get()));
	else if(i == 3)
		return ((uint8_t)((1<<PHOTOSENSOR_3) & PhotosensorDriver_get()));
	else if(i == 4)
		return ((uint8_t)((1<<PHOTOSENSOR_4) & PhotosensorDriver_get()));
	
	return 0;
}

/**
 * \fn	uint8_t PhotosensorInterface_is_first_bay_occupied()
 * \brief	Prüft, ob auf dem ersten Platz einer Rampe ein Paket liegt
 *
 * \author	Jan-Gerd Meß
 */
uint8_t PhotosensorInterface_is_first_bay_occupied(){
	return PhotosensorInterface_is_bay_occupied(1);
}

/**
 * \fn	uint8_t PhotosensorInterface_num_packages()
 * \brief	Zählt die Anzahl der Pakete auf der Rampe
 *
 * \author	Jan-Gerd Meß
 */
uint8_t PhotosensorInterface_num_packages(){
	uint8_t i = 1, n = 0;
	for(i = 0; i<5;i++)
		if(PhotosensorInterface_is_bay_occupied(i))
			n++;
	return n;
}