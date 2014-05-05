/**
 * \file photosensor_drv.c
 * \brief	Treiber auf Pin-Ebene f�r die Lichtschranken
 *
 * \author	Jan-Gerd Me�
 * \date    12.03.2014
 */ 
#include "drivers/photosensor_drv.h"


/**
 * \fn	void PhotosensorDriver_init(void)
 * \brief	Initialisiert den Lichtschranken-Treiber: Setzt die Ports f�r alle vier Lichtschranken als Eing�nge
 *
 * \author	Jan-Gerd Me�
 */
void PhotosensorDriver_init(void){
	PHOTOSENSORS_PxDIR &= 0x0F;
	PHOTOSENSORS_PxOUT |= 0xF0;
}


/**
 * \fn	uint8_t PhotosensorDriver_get()
 * \brief	Gibt den Zustand der Lichtschranken zur�ck. Format der R�ckgabe: X X X X LS4 LS3 LS2 LS1
 *
 * \author	Jan-Gerd Me�
 */
uint8_t PhotosensorDriver_get(){
	uint8_t res = 0xF0 & ~(PINC);
	return res;
}