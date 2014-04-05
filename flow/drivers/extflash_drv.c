/**
 * \file extflash_drv.c
 * \brief	Treiber auf Pin-Ebene für Zugriffe auf den externen Speicher
 *
 * \author	Jan-Gerd Meß
 * \date    24.03.2014
 */

#include "drivers/extflash_drv.h"

#define DEBUG 1

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/**
 * \fn	void extflash_init()
 * \brief	Initialisiert den Speicher-Treiber: Setzt die Ports für CS, CLK SO als Ausgänge und SI als Eingang
 *
 * \author	Jan-Gerd Meß
 */
void extflash_init(){
	// CLK und TXD auf LOW
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
	
	// CLK und TXD / SO auf Ausgang
	EXTFLASH_SPI_DDR |= 1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD;
	
	// SI / RXD auf Eingang
	EXTFLASH_SPI_DDR &= ~1<<EXTFLASH_RXD;
	
	// CS auf HIGH
	EXTFLASH_CS_PORT |= 1<<EXTFLASH_CS;
	
	// CS als Ausgang
	EXTFLASH_CS_DDR |= 1<<EXTFLASH_CS;
}

/**
 * \fn	void extflash_enable()
 * \brief	Aktiviert den Speicher, indem der Chipselect (low active) auf 0 gesetzt wird.
 *
 * \author	Jan-Gerd Meß
 */
void extflash_enable(){
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
	EXTFLASH_CS_PORT &= ~(1<<EXTFLASH_CS);
}

/**
 * \fn	void extflash_enable()
 * \brief	Deaktiviert den Speicher, indem der Chipselect (low active) auf 1 gesetzt wird.
 *
 * \author	Jan-Gerd Meß
 */		
void extflash_disable(){
	EXTFLASH_CS_PORT |= 1<<EXTFLASH_CS;
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK) | 1<<EXTFLASH_TXD;
}

/**
 * \fn	void extflash_wait_idle()
 * \brief	Wartet darauf, dass der Speicher eine 1 sendet, was bedeutet, dass er nicht mehr mit internen Operationen beschäftigt ist.
 *
 * \author	Jan-Gerd Meß
 */	
void extflash_wait_idle(){
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
	clock_wait(1);
	while(!(PIND & (1<<EXTFLASH_RXD))){
		// CLOCK TICK
		EXTFLASH_SPI_PORT |= 1<<EXTFLASH_CLK;
		EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK);
		clock_wait(1);
	}
}

/**
 * \fn	uint8_t extflash_tr_byte(uint8_t spiOut, uint8_t isRead)
 * \brief	Sendet ein Byte an den Speicher und gibt das gleichzeitig empfangene Byte zurück (SPI).
 *
 * \param spiOut Das zu sendende Byte
 * \param isRead Gibt an, ob auch ein Byte gelesen werden soll. In diesem Fall muss ein Takt gewartet werden, bis die Daten an SI / RXD gültig sind.
 *
 * \author	Jan-Gerd Meß
 */	
uint8_t extflash_tr_byte(uint8_t spiOut, uint8_t isRead){
	
	uint8_t recv = 0;
	uint8_t i = 8;

	PRINTF("SPI SENT: %u\n", spiOut);
	
	// Warte auf Operationen im Speicher
	extflash_wait_idle();
	
	// Starte Übertragung, MSB zuerst
	for(;i >= 1; i--){	
		// Prüfe i-tes Bit im Eingang
		if(spiOut & (1<<(i-1))){
			EXTFLASH_SPI_PORT |= 1<<EXTFLASH_TXD;
		} else {
			EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_TXD);
		}
		
		// CLOCK Tick
		EXTFLASH_SPI_PORT |= 1<<EXTFLASH_CLK;
		EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK);
		
		// Eventuell auf Gültigkeit des Ausgangs-Signals warten und recv setzen
		if(isRead){
			_NOP();
			_NOP();
			if((PIND & (1<<EXTFLASH_RXD))){
				recv |= 1<<(i-1);
			}
		}
	}
	PRINTF("RECEIVED: %u\n\n", recv);
	return recv;
}

/**
 * \fn	uint8_t extflash_read_status_register(void)
 * \brief	Gibt das Statusregister des Speichers zurück.
 *
 * \author	Jan-Gerd Meß
 */	
uint8_t extflash_read_status_register(void)
{
	int sreg;
	uint8_t status_register;
	
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_tr_byte(EXTFLASH_OP_RDST, 0);
	status_register = extflash_tr_byte(EXTFLASH_DONT_CARE, 1);
	
	SREG = sreg;
	
	return status_register;
}


/**
 * \fn	uint8_t extflash_get_last_compare(void)
 * \brief	Gibt zurück, ob der letzte Compare eines Buffers und des Speichers einen Diff ergeben hat (1) oder ob beide identisch waren (0)
 *
 * \author	Jan-Gerd Meß
 */	
uint8_t extflash_get_last_compare(void){
	return (extflash_read_status_register() & 1<<EXTFLASH_ST_COMPARE);
}