/**
 * \file photosensor_drv.h
 * \brief	Treiber auf Pin-Ebene f�r die Lichtschranken
 *
 * \author	Jan-Gerd Me�
 * \date    12.03.2014
 */ 


#ifndef PHOTOSENSOR_DRV_H_
#define PHOTOSENSOR_DRV_H_

#include "contiki-conf.h"

/** Data Direction Register f�r die Lichtschranken */
#define PHOTOSENSORS_PxDIR DDRC

/** Port f�r die Lichtschranken */
#define PHOTOSENSORS_PxOUT PORTC

/** Pin f�r die Lichtschranken */
#define PHOTOSENSORS_PxIN PINC;

/** Pin f�r Lichtschranke 1 */
#define PHOTOSENSOR_1 PINC4

/** Pin f�r Lichtschranke 2 */
#define PHOTOSENSOR_2 PINC5

/** Pin f�r Lichtschranke 3 */
#define PHOTOSENSOR_3 PINC6

/** Pin f�r Lichtschranke 4 */
#define PHOTOSENSOR_4 PINC7

void photosensor_drv_init(void);
uint8_t get_photosensors();



#endif /* PHOTOSENSOR_DRV_H_ */