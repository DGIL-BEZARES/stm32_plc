#ifndef BL_FLASH_H
#define BL_FLASH_H

#include "common-defines.h"

void bl_flash_erase_main_application(void);
void bl_flash_write(const uint32_t addr, const uint8_t *data,
                    const uint32_t length);

#endif // !BL_FLASH_H
