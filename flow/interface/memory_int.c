/**
 * \file memory_int.c
 * \brief	Memory Interface
 *
 * \author	Jan-Gerd Meﬂ
 * \date    28.04.2014
 */  

#include "interface/memory_int.h"

/**
 * \fn	void memory_init()
 * \brief	Initialisiert den Speicher
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_init(){
	extflash_init();
}

/**
 * \fn	void memory_restore(uint16_t cell)
 * \brief	Holt die angegebene Zelle aus dem Speicher und schreibt sie in den Buffer
 *
 * \param cell Ziel-Zelle
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_restore(uint16_t cell){
	extflash_write_page_to_buffer1(cell);
	extflash_write_page_to_buffer2(cell+1);
}

/**
 * \fn	void memory_flush(uint16_t cell)
 * \brief	Schreibt den Buffer an die angegebene Zelle im Speicher
 *
 * \param cell Ziel-Zelle
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_flush(uint16_t cell){
	extflash_write_buffer1_to_mem(cell);
	extflash_write_buffer2_to_mem(cell+1);
}

/**
 * \fn	void memory_agent_restore(uint8_t localAgentId)
 * \brief	Holt den Speicherbereich eines Agenten in den Buffer
 *
 * \param localAgentId lokale ID des Agenten, dessen Speicherbereich in den Buffer geschrieben werden soll
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_agent_restore(uint8_t localAgentId){
	memory_restore(FLASH_AGENT_PAGE+2*localAgentId);
}

/**
 * \fn	void memory_agent_flush(uint8_t localAgentId)
 * \brief	Sichert den Buffer eines Agenten im Speicher
 *
 * \param localAgentId lokale ID des Agenten, dessen Speicherbereich zur¸ck in den Speicher geschrieben werden soll
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_agent_flush(uint8_t localAgentId){
	memory_flush(FLASH_AGENT_PAGE+2*localAgentId);
}

/**
 * \fn	void memory_config_restore()
 * \brief	Holt die Config-Zelle in den Buffer
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_config_restore(){
	memory_restore(FLASH_CONFIG_PAGE);
}

/**
 * \fn	void memory_config_flush()
 * \brief	Sichert die Config-Zelle in den Speicher
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_config_flush(){
	memory_flush(FLASH_CONFIG_PAGE);
}

/**
 * \fn	void memory_write(uint16_t address, uint8_t* data, uint8_t len)
 * \brief	Schreibt in den Buffer
 *
 * \param address Zieladresse im Buffer
 * \param data Zeiger auf die Daten
 * \param len Anzahl zu schreibender Bytes
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_write(uint16_t address, uint8_t* data, uint8_t len){
	address &= 0x3FF;
	if(address & 0x0200){
		extflash_write_buffer1(address, len, data);
	} else {
		extflash_write_buffer2(address, len, data);
	}
}

/**
 * \fn	void memory_read(uint16_t address, uint8_t* data, uint8_t len)
 * \brief	Liest aus dem Buffer
 *
 * \param address Zieladresse im Buffer
 * \param data Zeiger auf die Daten
 * \param len Anzahl zu schreibender Bytes
 *
 * \author	Jan-Gerd Meﬂ
 */
void memory_read(uint16_t address, uint8_t* data, uint8_t len){
	address &= 0x3FF;
	if(address & 0x0200){
		extflash_read_buffer1(address, data, len);
		} else {
		extflash_read_buffer2(address, data, len);
	}
}