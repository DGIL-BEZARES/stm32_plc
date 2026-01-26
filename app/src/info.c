#include "core/firmware-info.h"

__attribute__((section(".firmware_info"))) firmware_info_t firmware_info = {
    .sentinel = FWINFO_SENTINEL,
    .device_id = DEVICE_ID,
    .version = 0xffffffff,
    .length = 0xffffffff,
    .crc32 = 0xffffffff,
    .reserverd0 = 0xffffffff,
    .reserverd1 = 0xffffffff,
    .reserverd2 = 0xffffffff,
    .reserverd3 = 0xffffffff,
    .reserverd4 = 0xffffffff,
};