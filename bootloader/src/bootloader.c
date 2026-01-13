#include "common-defines.h"
#include <libopencm3/stm32/memorymap.h>

#define BOOTLOADER_SIZE (0x4000U) // 16KB
#define APP_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE)

static void jump_to_app(void)
{
  typedef void (*pFunction)(void);

  uint32_t *reset_vector_entry = (uint32_t *)(APP_START_ADDR + 4U);
  uint32_t *reset_vector = (uint32_t *)(*reset_vector_entry);

  pFunction jump_fn = (pFunction)reset_vector;

  jump_fn();
}

int main(void)
{
  jump_to_app();

  return 0;
}
