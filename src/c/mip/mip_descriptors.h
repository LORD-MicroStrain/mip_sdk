#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "microstrain/serialization.h"
#include "mip_result.h"

#ifdef __cplusplus
namespace mip {
namespace C {
using ::microstrain::C::microstrain_serializer;
extern "C" {
#endif // __cplusplus

enum {
    MIP_INVALID_DESCRIPTOR_SET        = 0x00,
    MIP_DATA_DESCRIPTOR_SET_START     = 0x80,
    MIP_RESERVED_DESCRIPTOR_SET_START = 0xF0,

    MIP_INVALID_FIELD_DESCRIPTOR  = 0x00,
    MIP_REPLY_DESCRIPTOR          = 0xF1,
    MIP_RESERVED_DESCRIPTOR_START = 0xF0,
    MIP_RESPONSE_DESCRIPTOR_START = 0x80,

    MIP_SHARED_DATA_FIELD_DESCRIPTOR_START = 0xD0,
};

bool mip_is_valid_descriptor_set(uint8_t descriptor_set);
bool mip_is_data_descriptor_set(uint8_t descriptor_set);
bool mip_is_cmd_descriptor_set(uint8_t descriptor_set);
bool mip_is_reserved_descriptor_set(uint8_t descriptor_set);
bool mip_is_gnss_data_descriptor_set(uint8_t descriptor_set);

bool mip_is_valid_field_descriptor(uint8_t field_descriptor);
bool mip_is_cmd_field_descriptor(uint8_t field_descriptor);
bool mip_is_reply_field_descriptor(uint8_t field_descriptor);
bool mip_is_response_field_descriptor(uint8_t field_descriptor);
bool mip_is_reserved_cmd_field_descriptor(uint8_t field_descriptor);
bool mip_is_shared_data_field_descriptor(uint8_t field_descriptor);

enum mip_function_selector
{
    MIP_FUNCTION_WRITE = 0x01,
    MIP_FUNCTION_READ  = 0x02,
    MIP_FUNCTION_SAVE  = 0x03,
    MIP_FUNCTION_LOAD  = 0x04,
    MIP_FUNCTION_RESET = 0x05,
};
typedef enum mip_function_selector mip_function_selector;
void insert_mip_function_selector(microstrain_serializer* serializer, enum mip_function_selector self);
void extract_mip_function_selector(microstrain_serializer* serializer, enum mip_function_selector* self);


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus
