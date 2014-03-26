/**
 * \file bolt_drv.c
 * \brief	Treiber f¸r die Bolzen an den Rampen.
 *
 * \author	Jan-Gerd Meﬂ
 * \date      24.02.2014
 */

#include "drivers/bolt_drv.h"

/**
 * \fn	void bolt_drv_init(void)
 * \brief	Initialisiert den Bolzen-Treiber: Setzt Pins BOLT_1 und BOLT_2 als Ausg‰nge und anschlieﬂend auf HIGH.
 *
 * \author	Jan-Gerd Meﬂ
 */
void bolt_drv_init(void){
	BOLTS_PxDIR |= (1<<BOLT_1 | 1<<BOLT_2);
	BOLTS_PxOUT |= (1<<BOLT_1 | 1<<BOLT_2);
}

/**
 * \fn	void bolts_up(unsigned char boltv)
 * \brief	Nimmt einen char-Vektor, maskiert diesen und setzt alle Bolzen hoch, an deren Stelle eine 1 im Vektor steht
 *
 * \param boltv Vektor mit Bolzen - nur die untersten beiden Bits werden beachtet.
 *
 * \author	Jan-Gerd Meﬂ
 */
void bolts_up(unsigned char boltv){
	boltv &= 0x03;
	BOLTS_PxOUT |= boltv;
}

/**
 * \fn	void bolts_down(unsigned char boltv)
 * \brief	Nimmt einen char-Vektor, maskiert diesen und setzt alle Bolzen herunter, an deren Stelle eine 1 im Vektor steht
 *
 * \param boltv Vektor mit Bolzen - nur die untersten beiden Bits werden beachtet.
 *
 * \author	Jan-Gerd Meﬂ
 */
void bolts_down(unsigned char boltv){
	boltv &= 0x03;
	BOLTS_PxOUT &= ~boltv; 
}

/**
 * \fn	void bolts_toggle(unsigned char boltv)
 * \brief	Nimmt einen char-Vektor, maskiert diesen und toggelt alle Bolzen, an deren Stelle eine 1 im Vektor steht
 *
 * \param boltv Vektor mit Bolzen - nur die untersten beiden Bits werden beachtet.
 *
 * \author	Jan-Gerd Meﬂ
 */
void bolts_toggle(unsigned char boltv){
	boltv &= 0x03;
	if(boltv & (1<<BOLT_1)){
		BOLTS_PxOUT ^= (1<<BOLT_1);
	}
	if(boltv & (1<<BOLT_2)){
		BOLTS_PxOUT ^= (1<<BOLT_2);
	}
}

/**
 * \fn	uint8_t get_bolts(void)
 * \brief	Gibt den Bolt-Vektor zur¸ck. Alle Bolzen, an deren Stelle eine 1 steht, sind hochgefahren.
 *
 * \author	Jan-Gerd Meﬂ
 */
uint8_t get_bolts(void){
	return 0x03 & BOLTS_PxOUT;
}