#include "fifo.h"

bool fifo_init(Fifo* fifo, uint8_t* buf, size_t size)
{
    if (!fifo || !buf || size < 1 || size > UINT16_MAX)
    {
        return false;
    }
    fifo->buffer = buf;
    fifo->size = (uint16_t)size;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
    return true;
}

bool fifo_is_empty(Fifo* fifo)
{
    if (!fifo)
    {
        return true;
    }
    return fifo->count == 0;
}

bool fifo_is_full(Fifo* fifo)
{
    if (!fifo)
    {
        return true;
    }
    return fifo->count == fifo->size;
}

bool fifo_push(Fifo* fifo, uint8_t data)
{
    if (!fifo || !fifo->buffer || fifo_is_full(fifo))
    {
        return false;
    }
    fifo->buffer[fifo->head] = data;
    fifo->head = (fifo->head + 1) % fifo->size;
    fifo->count++;
    return true;
}

uint16_t fifo_push_bytes(Fifo* fifo, const uint8_t* data, uint16_t count)
{
    if (!fifo || !fifo->buffer || !data || count == 0)
    {
        return 0;
    }
    // Calculate how many bytes can be pushed
    uint16_t available = fifo->size - fifo->count;
    uint16_t to_push = count < available ? count : available;
    uint16_t pushed = 0;

    // Push bytes until wrap-around or count is reached
    while (pushed < to_push)
    {
        fifo->buffer[fifo->head] = data[pushed];
        fifo->head = (fifo->head + 1) % fifo->size;
        pushed++;
    }
    fifo->count += pushed;
    return pushed;
}

bool fifo_pop(Fifo* fifo, uint8_t* data)
{
    if (!fifo || !fifo->buffer || !data || fifo_is_empty(fifo))
    {
        return false;
    }
    *data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % fifo->size;
    fifo->count--;
    return true;
}

uint16_t fifo_pop_bytes(Fifo* fifo, uint8_t* data, uint16_t count)
{
    if (!fifo || !fifo->buffer || !data || count == 0 || fifo_is_empty(fifo))
    {
        return 0;
    }
    // Calculate how many bytes can be popped
    uint16_t to_pop = count < fifo->count ? count : fifo->count;
    uint16_t popped = 0;

    // Pop bytes until wrap-around or count is reached
    while (popped < to_pop)
    {
        data[popped] = fifo->buffer[fifo->tail];
        fifo->tail = (fifo->tail + 1) % fifo->size;
        popped++;
    }
    fifo->count -= popped;
    return popped;
}

uint16_t fifo_count(Fifo* fifo)
{
    if (!fifo)
    {
        return 0;
    }
    return fifo->count;
}

void fifo_reset(Fifo* fifo)
{
    if (fifo)
    {
        fifo->head = 0;
        fifo->tail = 0;
        fifo->count = 0;
    }
}