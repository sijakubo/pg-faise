/**
 * \file extflash_drv.c
 * \brief	Treiber auf Pin-Ebene f�r Zugriffe auf den externen Speicher
 *
 * \author	Jan-Gerd Me�
 * \date    24.03.2014
 */

#include "drivers/extflash_drv.h"
#define _NOP() asm volatile("nop")

/**
 * \fn	void ExtflashDriver_init()
 * \brief	Initialisiert den Speicher-Treiber: Setzt die Ports f�r CS, CLK SO als Ausg�nge und SI als Eingang
 *
 * \author	Jan-Gerd Me�
 */
void ExtflashDriver_init(){
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
 * \fn	void ExtflashDriver_enable()
 * \brief	Aktiviert den Speicher, indem der Chipselect (low active) auf 0 gesetzt wird.
 *
 * \author	Jan-Gerd Me�
 */
void ExtflashDriver_enable(){
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
	EXTFLASH_CS_PORT &= ~(1<<EXTFLASH_CS);
}

/**
 * \fn	void ExtflashDriver_disable()
 * \brief	Deaktiviert den Speicher, indem der Chipselect (low active) auf 1 gesetzt wird.
 *
 * \author	Jan-Gerd Me�
 */		
void ExtflashDriver_disable(){
	EXTFLASH_CS_PORT |= 1<<EXTFLASH_CS;
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK) | 1<<EXTFLASH_TXD;
}

/**
 * \fn	void ExtflashDriver_wait_idle()
 * \brief	Wartet darauf, dass der Speicher eine 1 sendet, was bedeutet, dass er nicht mehr mit internen Operationen besch�ftigt ist.
 *
 * \author	Jan-Gerd Me�
 */	
void ExtflashDriver_wait_idle(){
	while(!(extflash_read_status_register() & 0x80)){
		clock_delay(1);
	}
	extflash_enable();
}

/**
 * \fn	uint8_t ExtflashDriver_tr_byte(uint8_t spiOut, uint8_t isRead)
 * \brief	Sendet ein Byte an den Speicher und gibt das gleichzeitig empfangene Byte zur�ck (SPI).
 *
 * \param spiOut Das zu sendende Byte
 * \param isRead Gibt an, ob auch ein Byte gelesen werden soll. In diesem Fall muss ein Takt gewartet werden, bis die Daten an SI / RXD g�ltig sind.
 *
 * \author	Jan-Gerd Me�
 */	
uint8_t ExtflashDriver_tr_byte(uint8_t spiOut){
	
	uint8_t recv = 0;
	uint8_t i = 8;
	
	// Warte auf Operationen im Speicher
	//extflash_wait_idle();
	
	// Starte �bertragung, MSB zuerst
	for(;i >= 1; i--){	
		// Pr�fe i-tes Bit im Eingang
		if(spiOut & (1<<(i-1))){
			EXTFLASH_SPI_PORT |= 1<<EXTFLASH_TXD;
		} else {
			EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_TXD);
		}
		
		// CLOCK Tick
		EXTFLASH_SPI_PORT |= 1<<EXTFLASH_CLK;
		EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK);
		
		if((PIND & (1<<EXTFLASH_RXD))){
			recv |= 1<<(i-1);
		}
	}
	return recv;
}

/**
 * \fn	uint8_t ExtflashDriver_read_status_register(void)
 * \brief	Gibt das Statusregister des Speichers zur�ck.
 *
 * \author	Jan-Gerd Me�
 */	
uint8_t ExtflashDriver_read_status_register(void)
{
	int sreg;
	uint8_t status_register;
	
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_tr_byte(EXTFLASH_OP_RDST);
	status_register = extflash_tr_byte(EXTFLASH_DONT_CARE);
	
	SREG = sreg;
	extflash_disable();
	
	return status_register;
}


/**
 * \fn	uint8_t ExtflashDriver_get_last_compare(void)
 * \brief	Gibt zur�ck, ob der letzte Compare eines Buffers und des Speichers einen Diff ergeben hat (1) oder ob beide identisch waren (0)
 *
 * \author	Jan-Gerd Me�
 */	
uint8_t ExtflashDriver_get_last_compare(void){
	return (extflash_read_status_register() & 1<<EXTFLASH_ST_COMPARE);
}