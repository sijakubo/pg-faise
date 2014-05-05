/**
 * \file extflash_int.c
 * \brief	External Flash Interface
 *
 * \author	Jan-Gerd Meﬂ
 * \date    28.04.2014
 */  

#include "interface/extflash_int.h"

/**
 * \fn	void ExtflashInterface_init()
 * \brief	Initialisiert den Speicher
 *
 * \author	Jan-Gerd Meﬂ
 */
void ExtflashInterface_init(){
	ExtflashDriver_init();
}

/**
 * \fn	void ExtflashInterface_restore(uint16_t cell)
 * \brief	Holt die angegebene Zelle aus dem Speicher und schreibt sie in den Buffer
 *
 * \param cell Ziel-Zelle
 *
 * \author	Jan-Gerd Meﬂ
 */
void ExtflashInterface_restore(uint16_t cell){
	ExtflashService_write_page_to_buffer1(cell);
	ExtflashService_write_page_to_buffer2(cell+1);
}

/**
 * \fn	void ExtflashInterface_flush(uint16_t cell)
 * \brief	Schreibt den Buffer an die angegebene Zelle im Speicher
 *
 * \param cell Ziel-Zelle
 *
 * \author	Jan-Gerd Meﬂ
 */
void ExtflashInterface_flush(uint16_t cell){
	ExtflashService_write_buffer1_to_mem(cell);
	ExtflashService_write_buffer2_to_mem(cell+1);
}

/**
 * \fn	void ExtflashInterface_agent_restore(uint8_t localAgentId)
 * \brief	Holt den Speicherbereich eines Agenten in den Buffer
 *
 * \param localAgentId lokale ID des Agenten, dessen Speicherbereich in den Buffer geschrieben werden soll
 *
 * \author	Jan-Gerd Meﬂ
 */
void ExtflashInterface_agent_restore(uint8_t localAgentId){
	ExtflashInterface_restore(FLASH_AGENT_PAGE+2*localAgentId);
}

/**
 * \fn	void ExtflashInterface_agent_flush(uint8_t localAgentId)
 * \brief	Sichert den Buffer eines Agenten im Speicher
 *
 * \param localAgentId lokale ID des Agenten, dessen Speicherbereich zur¸ck in den Speicher geschrieben werden soll
 *
 * \author	Jan-Gerd Meﬂ
 */
void ExtflashInterface_agent_flush(uint8_t localAgentId){
	ExtflashInterface_flush(FLASH_AGENT_PAGE+2*localAgentId);
}

/**
 * \fn	uint8_t ExtflashInterface_write(uint16_t address, uint8_t* data, uint8_t len)
 * \brief	Schreibt in den Buffer
 *
 * \param address Zieladresse im Buffer
 * \param data Zeiger auf die Daten
 * \param len Anzahl zu schreibender Bytes
 *
 * \author	Jan-Gerd Meﬂ
 */
uint8_t ExtflashInterface_write(uint16_t address, uint8_t* data, uint8_t len){
	address &= 0x3FF;
	if((address&0x01FF)+len <= 0x107){
		if(address & 0x0200){
			ExtflashService_write_buffer1((address&0x01FF), len, data);
		} else {
			ExtflashService_write_buffer2(address, len, data);
		}
		return TRUE;
	}
	return FALSE;
}

/**
 * \fn	void ExtflashInterface_read(uint16_t address, uint8_t* data, uint8_t len)
 * \brief	Liest aus dem Buffer
 *
 * \param address Zieladresse im Buffer
 * \param data Zeiger auf die Daten
 * \param len Anzahl zu schreibender Bytes
 *
 * \return 1 iff memory was successfully read 
 *
 * \author	Jan-Gerd Meﬂ
 */
uint8_t ExtflashInterface_read(uint16_t address, uint8_t* data, uint8_t len){
	address &= 0x3FF;
	if((address&0x01FF)+len <= 0x107){
		if(address & 0x0200){
			ExtflashService_read_buffer1((address&0x01FF), data, len);
		} else {
			ExtflashService_read_buffer2(address, data, len);
		}
		return TRUE;
	}
	return FALSE;
}