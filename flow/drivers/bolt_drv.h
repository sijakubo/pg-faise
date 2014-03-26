/**
 * \file bolt_drv.h
 * \brief	Treiber für die Bolzen an den Rampen.
 *
 * \author	Jan-Gerd Meß
 * \date	24.02.2014
 */


#ifndef BOLT_DRV_H_
#define BOLT_DRV_H_

#include "contiki-conf.h"

/** PIN für Bolzen 1 */
#define BOLT_1_1 PINC0
#define BOLT_1_2 PINC1

/** PIN für Bolzen 2 */
#define BOLT_2_1 PINC2
#define BOLT_2_2 PINC3

/** Data Direction Register für die Bolzen - Bits BOLT_1 und BOLT_2 müssen auf Ausgang gesetzt werden */
#define BOLTS_PxDIR DDRC

/** Outport für die Bolzen */
#define BOLTS_PxOUT PORTC

void bolt_drv_init(void);
void bolts_up(unsigned char boltv);
void bolts_down(unsigned char boltv);
void bolts_toggle(unsigned char boltv);
uint8_t get_bolts(void);

#endif