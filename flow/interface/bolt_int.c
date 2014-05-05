/**
 * \file bolt_int.c
 * \brief	Interface für die Bolzen
 *
 * \author	Jan-Gerd Meß
 * \date    24.02.2014
 */ 

#include "sys/rtimer.h"
#include "drivers/bolt_drv.h"
#include "interface/bolt_int.h"

/** Prozess, der ein Paket ausgibt */
PROCESS(bolt_int_release, "Release Package"); 

/** Prozess, der ein Paket ausgibt und das nächste separiert */
PROCESS(bolt_int_release_and_separate, "Release and Separate Package");

/** Prozess, der ein Paket separiert */
PROCESS(bolt_int_separate, "Separate Package");

static struct etimer bolt_timer;
static uint8_t bolt_state = 0;

/**
 * \brief	Prozess für die Ausgabe eines Pakets. Zeitbasierter Zustandsautomat.
 *
 * \author	Jan-Gerd Meß
 */
PROCESS_THREAD(bolt_int_release, ev, data)
{
	uint16_t timeout = 0;
	
	PROCESS_BEGIN();
	
	bolt_state = 0;
	
	while(bolt_state < 2){
		if(bolt_state == 0){
			BoltDriver_up(BOLT_UPPER);
			timeout = BOLT_TIMEOUT;
		} else if(bolt_state == 1) {
			BoltDriver_down(BOLT_LOWER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		bolt_state++;
	}
	BoltDriver_up(BOLT_LOWER);
	PROCESS_END();
}

/**
 * \brief	Prozess für die Ausgabe und das Separieren eines Pakets. Zeitbasierter Zustandsautomat.
 *
 * \author	Jan-Gerd Meß
 */
PROCESS_THREAD(bolt_int_release_and_separate, ev, data)
{
	uint16_t timeout = 0;
	
	PROCESS_BEGIN();
	
	bolt_state = 0;
	
	while(bolt_state < 4){
		if(bolt_state == 0){
			BoltDriver_up(BOLT_UPPER);
			timeout = BOLT_TIMEOUT;
		} else if(bolt_state == 1) {
			BoltDriver_down(BOLT_LOWER);
			timeout = BOLT_SLIDE_TIME;
		} else if(bolt_state == 2) {
			BoltDriver_up(BOLT_LOWER);
			timeout = BOLT_TIMEOUT;
		} else if(bolt_state == 3) {
			BoltDriver_down(BOLT_UPPER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		bolt_state++;
	}
	BoltDriver_up(BOLT_UPPER);
	PROCESS_END();
}

/**
 * \brief	Prozess für das Separieren eines Pakets. Zeitbasierter Zustandsautomat.
 *
 * \author	Jan-Gerd Meß
 */
PROCESS_THREAD(bolt_int_separate, ev, data)
{
	uint16_t timeout = 0;
	
	PROCESS_BEGIN();
	
	bolt_state = 0;
	
	while(bolt_state < 2){
		if(bolt_state == 0){
			BoltDriver_up(BOLT_LOWER);
			timeout = BOLT_TIMEOUT;
		} else if(bolt_state == 1) {
			BoltDriver_down(BOLT_UPPER);
			timeout = BOLT_SLIDE_TIME;
		}
		etimer_set(&bolt_timer, timeout);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&bolt_timer));
		bolt_state++;
	}
	BoltDriver_up(BOLT_UPPER);
	PROCESS_END();
}

/**
 * \fn	void BoltInterface_release()
 * \brief	Startet den Release-Prozess
 *
 * \author	Jan-Gerd Meß
 */
void BoltInterface_release(){
	process_start(&bolt_int_release, NULL);
}

/**
 * \fn	void BoltInterface_release_and_separate()
 * \brief	Startet den Release-And-Separate-Prozess
 *
 * \author	Jan-Gerd Meß
 */
void BoltInterface_release_and_separate(){
	process_start(&bolt_int_release_and_separate, NULL);
}

/**
 * \fn	void BoltInterface_separate()
 * \brief	Startet den Separate-Prozess
 *
 * \author	Jan-Gerd Meß
 */
void BoltInterface_separate(){
	process_start(&bolt_int_separate, NULL);
}

/**
 * \fn	void BoltInterface_init()
 * \brief	Initialisiert die Bolzen
 *
 * \author	Jan-Gerd Meß
 */
void BoltInterface_init(void){
	BoltDriver_init();
}