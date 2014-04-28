/**
 * \file memory_int.h
 * \brief	Memory Interface
 *
 * \author	Jan-Gerd Meﬂ
 * \date    28.04.2014
 */  


#ifndef MEMORY_INT_H_
#define MEMORY_INT_H_


#define FLASH_CONFIG_PAGE 0x0100
#define FLASH_AGENT_PAGE 0x0101


void memory_init();

void memory_restore(uint16_t page);

void memory_flush(uint16_t page);

void memory_agent_restore(uint8_t localAgentId)

void memory_agent_flush(uint8_t localAgentId);

void memory_config_restore();

void memory_config_flush();

void memory_write(uint16_t address, uint8_t* data, uint8_t len);

void memory_read(uint16_t address, uint8_t* data, uint8_t len);


#endif /* MEMORY_INT_H_ */