/**
 * \file extflash_int.h
 * \brief	External Flash Interface
 *
 * \author	Jan-Gerd Meﬂ
 * \date    28.04.2014
 */  


#ifndef EXTFLASH_INT_H_
#define EXTFLASH_INT_H_

#include "contiki.h"
#include "defines/Defines.h"

#define FLASH_AGENT_PAGE 0x0100

void ExtflashInterface_init();

void ExtflashInterface_restore(uint16_t page);

void ExtflashInterface_flush(uint16_t page);

void ExtflashInterface_agent_restore(uint8_t localAgentId);

void ExtflashInterface_agent_flush(uint8_t localAgentId);

uint8_t ExtflashInterface_write(uint16_t address, uint8_t* data, uint8_t len);

uint8_t ExtflashInterface_read(uint16_t address, uint8_t* data, uint8_t len);


#endif /* EXTFLASH_INT_H_ */