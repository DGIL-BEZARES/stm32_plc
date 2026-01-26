#ifndef FIRMWARE_INFO_H
#define FIRMWARE_INFO_H

#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/flash.h>

#include "common-defines.h"

#define BOOTLOADER_SIZE (0x4000U) // 16KB
#define APP_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE)
#define MAX_FW_LENGTH ((1024U * 128U) - BOOTLOADER_SIZE)
#define DEVICE_ID (0X42)

#define FWINFO_ADDR (APP_START_ADDR + sizeof(vector_table_t))
#define FWINFO_VALIDATE_FROM (FWINFO_ADDR + sizeof(firmware_info_t))
#define FWINFO_VALIDATE_LENGHT(fw_length)                                      \
  (fw_length - (sizeof(vector_table_t) + sizeof(firmware_info_t)))

#define FWINFO_SENTINEL (0xDEADBEEF)

typedef struct firmware_info_t {
  uint32_t sentinel;
  uint32_t device_id;
  uint32_t version;
  uint32_t length;
  uint32_t crc32;
  uint32_t reserverd0;
  uint32_t reserverd1;
  uint32_t reserverd2;
  uint32_t reserverd3;
  uint32_t reserverd4;

} firmware_info_t;

#endif // FIRMWARE_INFO_H
