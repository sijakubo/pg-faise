/**
 * \file photosensor_drv.c
 * \brief	Treiber auf Pin-Ebene für die Lichtschranken
 *
 * \author	Jan-Gerd Meß
 * \date    12.03.2014
 */ 
#include "drivers/photosensor_drv.h"


/**
 * \fn	void PhotosensorDriver_init(void)
 * \brief	Initialisiert den Lichtschranken-Treiber: Setzt die Ports für alle vier Lichtschranken als Eingänge
 *
 * \author	Jan-Gerd Meß
 */
void PhotosensorDriver_init(void){
	PHOTOSENSORS_PxDIR &= 0x0F;
	PHOTOSENSORS_PxOUT |= 0xF0;
}


/**
 * \fn	uint8_t PhotosensorDriver_get()
 * \brief	Gibt den Zustand der Lichtschranken zurück. Format der Rückgabe: X X X X LS4 LS3 LS2 LS1
 *
 * \author	Jan-Gerd Meß
 */
uint8_t PhotosensorDriver_get(){
	uint8_t res = 0xF0 & ~(PINC);
	return res;
}