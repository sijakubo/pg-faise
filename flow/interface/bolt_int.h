/**
 * \file bolt_int.h
 * \brief	Interface für die Bolzen
 *
 * \author	Jan-Gerd Meß
 * \date    24.02.2014
 */ 


#ifndef BOLT_INT_H_
#define BOLT_INT_H_

#include "contiki.h"
#include "sys/clock.h"

// Zeit, die ein Paket benötigt, um die Rampe herunter zu rutschen
#define BOLT_SLIDE_TIME CLOCK_SECOND*2

// Zeit, bis der Bolzen geöffnet oder geschlossen ist
#define BOLT_TIMEOUT CLOCK_SECOND/2

// unterer Bolzen
#define BOLT_LOWER 0x01

// oberer Bolzen
#define BOLT_UPPER 0x02

void BoltInterface_init(void);
void BoltInterface_release(void);
void BoltInterface_release_and_separate(void);
void BoltInterface_separate(void);

PROCESS_NAME(bolt_int_release);
PROCESS_NAME(bolt_int_release_and_separate);
PROCESS_NAME(bolt_int_separate);

#endif /* BOLT_INT_H_ */