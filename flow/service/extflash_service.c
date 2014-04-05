/*
 * extflash_service.c
 *
 * Created: 26.03.2014 07:36:13
 *  Author: JanGerd
 */ 

#include "extflash_service.h"

void extflash_write_page_to_buffer1(uint16_t page){
	int sreg;
	
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_MEM2BUF1, 0);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7), 0);
	extflash_tr_byte((uint8_t)(0x01 | page << 1), 0);
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
}

void extflash_write_page_to_buffer2(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_MEM2BUF2, 0);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7), 0);
	extflash_tr_byte((uint8_t)(0x01 | page << 1), 0);
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
}

void extflash_write_buffer1(uint16_t address, uint8_t len, uint8_t* data){
	int sreg;
	address &= 0x01FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_WRBUF1, 0);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8), 0);
	extflash_tr_byte((uint8_t)address, 0);
	while(len > 0){
		extflash_tr_byte(*data, 0);
		data++;
		len--;
	}
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
}

void extflash_write_buffer2(uint16_t address, uint8_t len, uint8_t* data){
	int sreg;
	address &= 0x01FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_WRBUF2, 0);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8), 0);
	extflash_tr_byte((uint8_t)address, 0);
	while(len > 0){
		extflash_tr_byte(*data, 0);
		data++;
		len--;
	}
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
}

uint8_t extflash_compare_buffer1_to_mem(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_BUF1CMP, 0);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7), 0);
	extflash_tr_byte((uint8_t)(0x01 | page << 1), 0);
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
	
	return extflash_get_last_compare();
}

uint8_t extflash_compare_buffer2_to_mem(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_BUF2CMP, 0);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7), 0);
	extflash_tr_byte((uint8_t)(0x01 | page << 1), 0);
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
	
	return extflash_get_last_compare();
}

void extflash_write_buffer1_to_mem(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_BUF12MEM, 0);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7), 0);
	extflash_tr_byte((uint8_t)(0x01 | page << 1), 0);
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
}

void extflash_write_buffer2_to_mem(uint16_t page){
	int sreg;
	page &= 0x07FF;
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	extflash_tr_byte(EXTFLASH_OP_BUF22MEM, 0);
	extflash_tr_byte((uint8_t)(0xf0 | page >> 7), 0);
	extflash_tr_byte((uint8_t)(0x01 | page << 1), 0);
	extflash_tr_byte(EXTFLASH_DONT_CARE, 0);
	
	extflash_disable();
	
	SREG = sreg;
}

void exflash_read_buffer1(uint16_t address, uint8_t* buffer, uint8_t len){
	int sreg;
	address &= 0x01FF;
	
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_tr_byte(EXTFLASH_OP_RDBUF1, 0);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8), 0);
	extflash_tr_byte((uint8_t)address, 0);
	while(len > 0){
		*buffer = extflash_tr_byte(EXTFLASH_DONT_CARE, 1);
		buffer =  buffer+1;
		len--;
	}
	
	extflash_disable();
	
	SREG = sreg;
}

void extflash_read_buffer2(uint16_t address, uint8_t* buffer, uint8_t len){
	int sreg;
	address &= 0x01FF;
	
	sreg = SREG;
	cli();
	
	extflash_enable();
	extflash_tr_byte(EXTFLASH_OP_RDBUF2, 0);
	extflash_tr_byte((uint8_t)(0x01 & address >> 8), 0);
	extflash_tr_byte((uint8_t)address, 0);
	while(len > 0){
		*buffer = extflash_tr_byte(EXTFLASH_DONT_CARE, 1);
		buffer =  buffer+1;
		len--;
	}
	
	extflash_disable();
	
	SREG = sreg;
}