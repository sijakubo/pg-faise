/**
 * \file photosensor_drv.h
 * \brief	Treiber auf Pin-Ebene für die Lichtschranken
 *
 * \author	Jan-Gerd Meß
 * \date    12.03.2014
 */ 


#ifndef PHOTOSENSOR_DRV_H_
#define PHOTOSENSOR_DRV_H_

#include "contiki-conf.h"

/** Data Direction Register für die Lichtschranken */
#define PHOTOSENSORS_PxDIR DDRC

/** Port für die Lichtschranken */
#define PHOTOSENSORS_PxOUT PORTC

/** Pin für die Lichtschranken */
#define PHOTOSENSORS_PxIN PINC;

/** Pin für Lichtschranke 1 */
#define PHOTOSENSOR_1 PINC4

/** Pin für Lichtschranke 2 */
#define PHOTOSENSOR_2 PINC5

/** Pin für Lichtschranke 3 */
#define PHOTOSENSOR_3 PINC6

/** Pin für Lichtschranke 4 */
#define PHOTOSENSOR_4 PINC7

void photosensor_drv_init(void);
uint8_t get_photosensors();



#endif /* PHOTOSENSOR_DRV_H_ */