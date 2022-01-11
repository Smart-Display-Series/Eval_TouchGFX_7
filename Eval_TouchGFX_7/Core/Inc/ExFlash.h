#ifndef EX_FLASH_H
#define EX_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

///////////////////////////////////////


typedef struct{
	
	enum{
		no_flash = 0,
		w25q128,
		mx25l25645g,
	}type;
	
	struct{
		uint32_t page;
		uint32_t subsector;
		uint32_t sector;
		uint32_t total;
	}size;
	
}flash_info_t;

///////////////////////////////////////

void ex_flash_init ( void );

extern flash_info_t flash_info;

#endif /* EX_FLASH_H*/
