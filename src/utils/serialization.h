#pragma once

#include <stdint.h>
#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////
///@defgroup Serialization - Functions for reading and writing to byte buffers.
///
///@{

void pack_bool(uint8_t* buffer, bool value);

void pack_u8 (uint8_t* buffer, uint8_t  value);
void pack_u16(uint8_t* buffer, uint16_t value);
void pack_u32(uint8_t* buffer, uint32_t value);
void pack_u64(uint8_t* buffer, uint64_t value);

void pack_s8 (uint8_t* buffer, int8_t   value);
void pack_s16(uint8_t* buffer, int16_t  value);
void pack_s32(uint8_t* buffer, int32_t  value);
void pack_s64(uint8_t* buffer, int64_t  value);

void pack_float (uint8_t* buffer, float  value);
void pack_double(uint8_t* buffer, double value);


void append_bool(uint8_t** buffer, bool value);

void append_u8 (uint8_t** buffer, uint8_t  value);
void append_u16(uint8_t** buffer, uint16_t value);
void append_u32(uint8_t** buffer, uint32_t value);
void append_u64(uint8_t** buffer, uint64_t value);

void append_s8 (uint8_t** buffer, int8_t   value);
void append_s16(uint8_t** buffer, int16_t  value);
void append_s32(uint8_t** buffer, int32_t  value);
void append_s64(uint8_t** buffer, int64_t  value);

void append_float (uint8_t** buffer, float  value);
void append_double(uint8_t** buffer, double value);




bool unpack_bool(const uint8_t* buffer);

uint8_t  unpack_u8 (const uint8_t* buffer);
uint16_t unpack_u16(const uint8_t* buffer);
uint32_t unpack_u32(const uint8_t* buffer);
uint64_t unpack_u64(const uint8_t* buffer);

int8_t   unpack_s8 (const uint8_t* buffer);
int16_t  unpack_s16(const uint8_t* buffer);
int32_t  unpack_s32(const uint8_t* buffer);
int64_t  unpack_s64(const uint8_t* buffer);

float  unpack_float (const uint8_t* buffer);
double unpack_double(const uint8_t* buffer);


bool extract_bool(const uint8_t** buffer);

uint8_t  extract_u8 (const uint8_t** buffer);
uint16_t extract_u16(const uint8_t** buffer);
uint32_t extract_u32(const uint8_t** buffer);
uint64_t extract_u64(const uint8_t** buffer);

int8_t   extract_s8 (const uint8_t** buffer);
int16_t  extract_s16(const uint8_t** buffer);
int32_t  extract_s32(const uint8_t** buffer);
int64_t  extract_s64(const uint8_t** buffer);

float  extract_float (const uint8_t** buffer);
double extract_double(const uint8_t** buffer);


///@}
////////////////////////////////////////////////////////////////////////////////
