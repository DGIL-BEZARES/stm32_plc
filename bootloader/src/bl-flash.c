#include "bl-flash.h"
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/memorymap.h>

#define MAIN_APP_PAGE_START (16)
#define MAIN_APP_PAGE_END (127)
#define PAGE_OFFSET (0x400U)

void bl_flash_erase_main_application(void) {
  flash_unlock();

  for (uint8_t page = MAIN_APP_PAGE_START; page <= MAIN_APP_PAGE_END; page++) {
    uint32_t page_addr = FLASH_BASE + (page * PAGE_OFFSET);
    flash_erase_page(page_addr);
  }

  flash_lock();
}

void bl_flash_write(const uint32_t addr, const uint8_t *data,
                    const uint32_t length) {
  flash_unlock();

  uint32_t num_elements = length / sizeof(uint32_t);

  for (uint32_t iter = 0; iter < num_elements; iter += 1) {
    flash_program_word(addr + (iter * 4), *((uint32_t *)(data + (iter * 4))));
  }

  flash_lock();
}
