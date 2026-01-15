#ifndef CRC8_H
#define CRC8_H

#include "common-defines.h"

#define CRC8_H2F_POLY (0x2F)
#define CRC8_H2F_INIT (0xFF)
#define CRC8_H2F_XOROUT (0xFF)

uint8_t crc8_h2f(const uint8_t *data, uint32_t len);
uint8_t crc8_h2f_lut(const uint8_t *data, uint32_t len);

#endif // CRC8_H
