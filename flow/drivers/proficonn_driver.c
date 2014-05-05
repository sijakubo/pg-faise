/**
 * \file proficonn_drv.c
 * \brief	Treiber auf Pin-Ebene für den Proficon-Adapter
 *
 * \author	Jan-Gerd Meß
 * \date    14.04.2014
 */ 

#include "drivers/proficonn_driver.h"

#define _NOP() asm volatile("nop")

/**
 * \fn	void ProfibusDriver_enable()
 * \brief	Aktiviert den Speicher, indem der Chipselect (low active) auf 0 gesetzt wird.
 *
 * \author	Jan-Gerd Meß
 */
void ProfibusDriver_init(){
	// CLK und TXD auf LOW
	PROFICON_SPI_PORT &= ~(1<<PROFICON_CLK | 1<<PROFICON_TXD);
	
	// CLK und TXD / SO auf Ausgang
	PROFICON_SPI_DDR |= 1<<PROFICON_CLK | 1<<PROFICON_TXD;
	
	// SI / RXD auf Eingang
	PROFICON_SPI_DDR &= ~1<<PROFICON_RXD;
	
	// CS auf HIGH
	PROFICON_CONF_PORT |= 1<<PROFICON_CS;
	// RESET auf Low
	PROFICON_CONF_PORT &= ~(1<<PROFICON_RESET);
	
	// CS und Reset als Ausgang
	PROFICON_CONF_DDR |= (1<<PROFICON_CS | 1 << PROFICON_RESET);
	// IRQ als Eingang
	PROFICON_CONF_DDR &= ~(1<<PROFICON_IRQ);
}

/**
 * \fn	void proficonn_enable()
 * \brief	Aktiviert den proficonn, indem der Chipselect (low active) auf 0 gesetzt wird.
 *
 * \author	Jan-Gerd Meß
 */
void ProfibusDriver_enable(){
	PROFICON_SPI_PORT &= ~(1<<PROFICON_CLK | 1<<PROFICON_TXD);
	PROFICON_CONF_PORT &= ~(1<<PROFICON_CS);
}

/**
 * \fn	void proficon_disable()
 * \brief	Deaktiviert den proficonn, indem der Chipselect (low active) auf 1 gesetzt wird.
 *
 * \author	Jan-Gerd Meß
 */		
void ProfibusDriver_disable(){
	PROFICON_CONF_PORT |= 1<<PROFICON_CS;
	PROFICON_SPI_PORT &= ~(1<<PROFICON_CLK | 1<<PROFICON_TXD);
}

/**
 * \fn	uint8_t proficon_tr_byte(uint8_t spiOut, uint8_t isRead)
 * \brief	Sendet ein Byte an den proficonn und gibt das gleichzeitig empfangene Byte zurück (SPI).
 *
 * \param spiOut Das zu sendende Byte
 * \param isRead Gibt an, ob auch ein Byte gelesen werden soll. In diesem Fall muss ein Takt gewartet werden, bis die Daten an SI / RXD gültig sind.
 *
 * \author	Jan-Gerd Meß
 */	
uint8_t ProfibusDriver_tr_byte(uint8_t spiOut){

	uint8_t recv = 0;
	uint8_t i = 8;
	
	// Starte Übertragung, MSB zuerst
	for(;i >= 1; i--){	
		// Prüfe i-tes Bit im Eingang
		if(spiOut & (1<<(i-1))){
			PROFICON_SPI_PORT |= 1<<PROFICON_TXD;
		} else {
			PROFICON_SPI_PORT &= ~(1<<PROFICON_TXD);
		}
		
		// CLOCK Tick
		PROFICON_SPI_PORT |= 1<<PROFICON_CLK;
		PROFICON_SPI_PORT &= ~(1<<PROFICON_CLK);
		
		if((PIND & (1<<PROFICON_RXD))){
			recv |= 1<<(i-1);
		}
	}
	return recv;
}

/**
 * \fn	void proficon_is_irq()
 * \brief	Gibt 1 zurück, wenn der proficonn einen Interrupt beantragt
 *
 * \author	Jan-Gerd Meß
 */	
uint8_t ProfibusDriver_is_irq(){
	return PINB & (1<<PROFICON_IRQ);
}

/**
 * \fn	void proficon_reset()
 * \brief	Reset für den proficonn
 *
 * \author	Jan-Gerd Meß
 */	
void proficonn_reset(){
	uint8_t i = 0;
	PROFICON_CONF_PORT |= 1 << PROFICON_RESET;
	for(;i<10;i++)
		_NOP();
	PROFICON_CONF_PORT &= ~(1<< PROFICON_RESET);
}