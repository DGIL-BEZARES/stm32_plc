#include "core/ring-buffer.h"

void ring_buffer_setup(ring_buffer_t *rb, uint8_t *pData, uint32_t size)
{
    rb->pDataBuffer = pData;
    rb->read_inex = 0;
    rb->write_index = 0;
    rb->mask = size - 1;
}

bool ring_buffer_empty(ring_buffer_t *rb)
{
    return (rb->read_inex == rb->write_index);
}

bool ring_buffer_read(ring_buffer_t *rb, uint8_t *pByte)
{
    uint32_t local_read_idx = rb->read_inex;

    if (!ring_buffer_empty(rb))
    {
        *pByte = rb->pDataBuffer[local_read_idx];
        local_read_idx = (local_read_idx + 1) & rb->mask;
        rb->read_inex = local_read_idx;
        return true;
    }
    return false;
}

bool ring_buffer_write(ring_buffer_t *rb, uint8_t byte)
{
    uint32_t local_read_idx = rb->read_inex;
    uint32_t local_write_idx = rb->write_index;

    uint32_t next_write_idx = (local_write_idx + 1) & rb->mask;

    if (next_write_idx == local_read_idx)
    {
        return false;
    }

    rb->pDataBuffer[local_write_idx] = byte;
    rb->write_index = next_write_idx;
    return true;
}
