#pragma once

#include <stdint.h>
#include <stddef.h>


struct ByteRingState
{
    uint8_t* buffer;
    size_t   size;
    size_t   head;
    size_t   tail;
};

void ByteRing_init(struct ByteRingState* state, uint8_t* buffer, size_t size);
void ByteRing_clear(struct ByteRingState* state);

size_t ByteRing_capacity(const struct ByteRingState* state);
size_t ByteRing_count(const struct ByteRingState* state);
size_t ByteRing_freeSpace(const struct ByteRingState* state);

uint8_t ByteRing_at(const struct ByteRingState* state, size_t index);

size_t ByteRing_pop(struct ByteRingState* state, size_t count);

size_t ByteRing_copyTo(const struct ByteRingState* state, uint8_t* buffer, size_t count);
size_t ByteRing_copyFromAndUpdate(struct ByteRingState* state, const uint8_t** bytes, size_t* count);
