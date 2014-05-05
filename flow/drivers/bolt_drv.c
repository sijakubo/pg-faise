/**
 * \file bolt_drv.c
 * \brief	Treiber f¸r die Bolzen an den Rampen.
 *
 * \author	Jan-Gerd Meﬂ
 * \date      24.02.2014
 */

#include "drivers/bolt_drv.h"
#include "sys/clock.h"

/**
 * \fn	void BoltDriver_init(void)
 * \brief	Initialisiert den Bolzen-Treiber: Setzt Pins BOLT_1 und BOLT_2 als Ausg‰nge und anschlieﬂend auf HIGH.
 *
 * \author	Jan-Gerd Meﬂ
 */
void BoltDriver_init(void){
	BOLTS_PxDIR |= (1<<BOLT_1_1 | 1<<BOLT_1_2 | 1<<BOLT_2_1 | 1<<BOLT_2_2);
	BOLTS_PxOUT |= ~(1<<BOLT_1_1 | 1<<BOLT_1_2 | 1<<BOLT_2_1 | 1 << BOLT_2_2);
}

/**
 * \fn	void BoltDriver_up(unsigned char boltv)
 * \brief	Nimmt einen char-Vektor, maskiert diesen und setzt alle Bolzen hoch, an deren Stelle eine 1 im Vektor steht
 *
 * \param boltv Vektor mit Bolzen - nur die untersten beiden Bits werden beachtet.
 *
 * \author	Jan-Gerd Meﬂ
 */
void BoltDriver_down(unsigned char boltv){
	boltv &= 0x03;
	if(boltv & 0x01){
		BOLTS_PxOUT |= 1<<BOLT_1_2;	
		BOLTS_PxOUT |= 1<<BOLT_1_1;	
		clock_wait(60);
		BOLTS_PxOUT &= ~(1<<BOLT_1_2);
	}
	if((boltv & 0x01) && (boltv & 0x02)){
		clock_wait(60);
	}
	if(boltv & 0x02){
		BOLTS_PxOUT |= 1<<BOLT_2_2;
		BOLTS_PxOUT |= 1<<BOLT_2_1;
		clock_wait(60);
		BOLTS_PxOUT &= ~(1<<BOLT_2_2);
	}
}

/**
 * \fn	void BoltDriver_up(unsigned char boltv)
 * \brief	Nimmt einen char-Vektor, maskiert diesen und setzt alle Bolzen herunter, an deren Stelle eine 1 im Vektor steht
 *
 * \param boltv Vektor mit Bolzen - nur die untersten beiden Bits werden beachtet.
 *
 * \author	Jan-Gerd Meﬂ
 */
void BoltDriver_up(unsigned char boltv){
	boltv &= 0x03;
	if(boltv & 0x01){
		BOLTS_PxOUT &= ~(1<<BOLT_1_1 | 1<<BOLT_1_2);
	}	
	if(boltv & 0x02){
		BOLTS_PxOUT &= ~(1<<BOLT_2_1 | 1<<BOLT_2_2);
	}
}

/**
 * \fn	uint8_t BoltDriver_get(void)
 * \brief	Gibt den Bolt-Vektor zur¸ck. Alle Bolzen, an deren Stelle eine 1 steht, sind hochgefahren.
 *
 * \author	Jan-Gerd Meﬂ
 */
uint8_t BoltDriver_get(void){
	uint8_t res = 0;
	if(BOLTS_PxOUT & 1<<BOLT_1_1)
		res |= 0x01;
	if(BOLTS_PxOUT & 1<<BOLT_2_1)
		res |= 0x02;
	return res;
}