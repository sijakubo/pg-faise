/**
 * \file extflash_service.c
 * \brief	Service für den externen Flashspeicher
 *
 * \author	Jan-Gerd Meß
 * \date    24.02.2014
 */ 

#include "extflash_service.h"

/**
 * \fn	void ExtflashService_write_page_to_buffer1(uint16_t page)
 * \brief	Schreibt eine Seite aus dem Speicher in Buffer 1
 *
 * \param page Die zu übertragende Seite
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_write_page_to_buffer1(uint16_t page){
	int sreg;
	
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_MEM2BUF1);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7));
	extflash_tr_byte((uint8_t)(0x01 | page << 1));
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	
	extflash_disable();
	
	SREG = sreg;
}


/**
 * \fn	void ExtflashService_write_page_to_buffer2(uint16_t page)
 * \brief	Schreibt eine Seite aus dem Speicher in Buffer 2
 *
 * \param page Die zu übertragende Seite
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_write_page_to_buffer2(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_MEM2BUF2);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7));
	extflash_tr_byte((uint8_t)(0x01 | page << 1));
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	
	extflash_disable();
	
	SREG = sreg;
}

/**
 * \fn	void ExtflashService_write_buffer1(uint16_t address, uint8_t len, uint8_t* data)
 * \brief	Schreibt in Buffer 1
 *
 * \param address Adresse im Buffer zu übertragende Seite
 * \param len Anzahl der zu schreibenden Bytes
 * \param data Startadresse der zu schreibenden Bytes
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_write_buffer1(uint16_t address, uint8_t len, uint8_t* data){
	int sreg;
	address &= 0x01FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_WRBUF1);
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8));
	extflash_tr_byte((uint8_t)address);
	while(len > 0){
		extflash_tr_byte(*data);
		data++;
		len--;
	}
	
	extflash_disable();
	
	SREG = sreg;
}

/**
 * \fn	void ExtflashService_write_buffer2(uint16_t address, uint8_t len, uint8_t* data)
 * \brief	Schreibt in Buffer 2
 *
 * \param address Adresse im Buffer zu übertragende Seite
 * \param len Anzahl der zu schreibenden Bytes
 * \param data Startadresse der zu schreibenden Bytes
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_write_buffer2(uint16_t address, uint8_t len, uint8_t* data){
	int sreg;
	address &= 0x01FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_WRBUF2);
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8));
	extflash_tr_byte((uint8_t)address);
	while(len > 0){
		extflash_tr_byte(*data);
		data++;
		len--;
	}
	
	extflash_disable();
	
	SREG = sreg;
}

/**
 * \fn	void ExtflashService_write_buffer1_to_mem(uint16_t page)
 * \brief	Schreibt Buffer 1 auf eine Seite in den Speicher
 *
 * \param page Ziel-Seite
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_write_buffer1_to_mem(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_BUF12MEM);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7));
	extflash_tr_byte((uint8_t)(0x01 | page << 1));
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	
	extflash_disable();
	
	SREG = sreg;
}

/**
 * \fn	void ExtflashService_write_buffer2_to_mem(uint16_t page)
 * \brief	Schreibt Buffer 2 auf eine Seite in den Speicher
 *
 * \param page Ziel-Seite
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_write_buffer2_to_mem(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_BUF22MEM);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7));
	extflash_tr_byte((uint8_t)(0x01 | page << 1));
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	
	extflash_disable();
	
	SREG = sreg;
}

/**
 * \fn	void ExtflashService_read_buffer1(uint16_t address, uint8_t* buffer, uint8_t len)
 * \brief	Liest aus Buffer 1
 *
 * \param address Startadresse im Buffer
 * \param buffer Adresse, an die die gelesenen Bytes geschrieben werden sollen
 * \param len Anzahl der zu lesenden BYtes
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_read_buffer1(uint16_t address, uint8_t* buffer, uint8_t len){
	int sreg;
	address &= 0x01FF;
	
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_RDBUF1);
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8));
	extflash_tr_byte((uint8_t)address);
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	while(len > 0){
		*buffer = extflash_tr_byte(EXTFLASH_DONT_CARE);
		buffer =  buffer+1;
		len--;
	}
	
	extflash_disable();
	
	SREG = sreg;
}

/**
 * \fn	void ExtflashService_read_buffer2(uint16_t address, uint8_t* buffer, uint8_t len)
 * \brief	Liest aus Buffer 2
 *
 * \param address Startadresse im Buffer
 * \param buffer Adresse, an die die gelesenen Bytes geschrieben werden sollen
 * \param len Anzahl der zu lesenden BYtes
 *
 * \author	Jan-Gerd Meß
 */
void ExtflashService_read_buffer2(uint16_t address, uint8_t* buffer, uint8_t len){
	int sreg;
	address &= 0x01FF;
	
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_wait_idle();
	extflash_tr_byte(EXTFLASH_OP_RDBUF2);
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8));
	extflash_tr_byte((uint8_t)address);
	extflash_tr_byte(EXTFLASH_DONT_CARE);
	while(len > 0){
		*buffer = extflash_tr_byte(EXTFLASH_DONT_CARE);
		buffer =  buffer+1;
		len--;
	}
	
	extflash_disable();
	
	SREG = sreg;
}