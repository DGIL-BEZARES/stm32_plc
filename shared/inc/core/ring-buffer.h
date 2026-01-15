#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "common-defines.h"

typedef struct ring_buffer_t
{
    uint8_t *pDataBuffer;
    uint32_t mask;
    uint32_t read_inex;
    uint32_t write_index;

} ring_buffer_t;

void ring_buffer_setup(ring_buffer_t *rb, uint8_t *pData, uint32_t size);
bool ring_buffer_empty(ring_buffer_t *rb);
bool ring_buffer_write(ring_buffer_t *rb, uint8_t byte);
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *pByte);

#endif // RING_BUFFER_H
