
#include "byte_ring.h"

#include <assert.h>


void ByteRing_init(struct ByteRingState* state, uint8_t* buffer, size_t size)
{
    assert( ((size - 1) & size) == 0 );  // Size must be a power of 2

    state->buffer = buffer;
    state->size   = size;
    state->head   = 0;
    state->tail   = 0;
}

void ByteRing_clear(struct ByteRingState* state)
{
    state->head = 0;
    state->tail = 0;
}

size_t ByteRing_capacity(const struct ByteRingState* state)
{
    return state->size;
}

size_t ByteRing_count(const struct ByteRingState* state)
{
    assert(state->head >= state->tail);

    return state->head - state->tail;
}

size_t ByteRing_freeSpace(const struct ByteRingState* state)
{
    size_t count = ByteRing_count(state);
    size_t size  = ByteRing_capacity(state);

    assert( count <= size );

    return size - count;
}

uint8_t ByteRing_at(const struct ByteRingState* state, size_t index)
{
    assert(state->head >= (state->tail+index));

    return state->buffer[ (state->tail + index) % state->size ];
}

size_t ByteRing_pop(struct ByteRingState* state, size_t count)
{
    size_t available = ByteRing_count(state);
    if( count > available )
        count = available;

    state->tail += count;

    return count;
}

size_t ByteRing_copyTo(const struct ByteRingState* state, uint8_t* buffer, size_t count)
{
    const size_t available = ByteRing_count(state);
    if( available < count )
        count = available;

    // const size_t count_A =
    for(size_t i=0; i<count; i++)
        buffer[i] = ByteRing_at(state, i);

    return count;
}

size_t ByteRing_copyFromAndUpdate(struct ByteRingState* state, const uint8_t** bytes, size_t* available)
{
    const size_t space = ByteRing_freeSpace(state);
    const size_t count = (*available < space) ? *available : space;

    const size_t capacity = ByteRing_capacity(state);

    for(size_t i=0; i<count; i++)
        state->buffer[ (state->head + i) % capacity ] = (*bytes)[i];

    state->head += count;

    *bytes += count;
    *available -= count;
    return count;
}