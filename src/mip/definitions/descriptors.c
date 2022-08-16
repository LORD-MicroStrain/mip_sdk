
#include "descriptors.h"

#include "../utils/serialization.h"

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set is valid.
///
///@param descriptor_set
///
///@returns true if the descriptor set is valid.
///
bool is_valid_descriptor_set(uint8_t descriptor_set)
{
    return descriptor_set != MIP_INVALID_DESCRIPTOR_SET;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set represents some kind of data.
///
///@param descriptor_set
///
///@returns true if the descriptor set represents data.
///
bool is_data_descriptor_set(uint8_t descriptor_set)
{
    return (descriptor_set >= MIP_DATA_DESCRIPTOR_SET_START) && (descriptor_set < MIP_RESERVED_DESCRIPTOR_SET_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set contains commands.
///
///@param descriptor_set
///
///@returns true if the descriptor set contains commands.
///
bool is_cmd_descriptor_set(uint8_t descriptor_set)
{
    return (descriptor_set < MIP_DATA_DESCRIPTOR_SET_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor is reserved for special purposes.
///
///@param descriptor_set
///
///@returns true if the descriptor set is reserved.
///
bool is_reserved_descriptor_set(uint8_t descriptor_set)
{
    return (descriptor_set >= MIP_RESERVED_DESCRIPTOR_SET_START);
}



////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is valid.
///
///@param field_descriptor
///
///@returns true if the field descriptor is valid.
///
bool is_valid_descriptor(uint8_t field_descriptor)
{
    return field_descriptor != MIP_INVALID_FIELD_DESCRIPTOR;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is a command.
///
///@param field_descriptor
///
///@returns true if the field descriptor represents a command.
///
bool is_command_descriptor(uint8_t field_descriptor)
{
    return (field_descriptor < MIP_RESPONSE_DESCRIPTOR_START);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is for an ack/nack reply.
///
///@param field_descriptor
///
///@returns true if the field descriptor represents an ack/nack reply code.
///
bool is_reply_descriptor(uint8_t field_descriptor)
{
    return (field_descriptor == MIP_REPLY_DESCRIPTOR);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor contains response data from a
///       command.
///
/// The descriptor set is assumed to be a command set.
///
///@param field_descriptor
///
///@returns true if the associated field contains response data.
///
bool is_response_descriptor(uint8_t field_descriptor)
{
    return field_descriptor >= MIP_RESPONSE_DESCRIPTOR_START && !is_reserved_descriptor(field_descriptor);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field descriptor is reserved.
///
/// The descriptor set is assumed to be a command set.
///
///@param field_descriptor
///
///@returns true if the associated field is neither a command nor response.
///
bool is_reserved_descriptor(uint8_t field_descriptor)
{
    return ((field_descriptor|MIP_RESPONSE_DESCRIPTOR_START) >= MIP_RESERVED_DESCRIPTOR_START);
}



void insert_mip_function_selector(struct mip_serializer* serializer, enum mip_function_selector self)
{
    return insert_u8(serializer, self);
}

void extract_mip_function_selector(struct mip_serializer* serializer, enum mip_function_selector* self)
{
    uint8_t tmp;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


void insert_mip_descriptor_rate(struct mip_serializer* serializer, const struct mip_descriptor_rate* self)
{
    insert_u8(serializer, self->descriptor);
    insert_u16(serializer, self->decimation);
}

void extract_mip_descriptor_rate(struct mip_serializer* serializer, struct mip_descriptor_rate* self)
{
    extract_u8(serializer, &self->descriptor);
    extract_u16(serializer, &self->decimation);
}


#ifdef __cplusplus
} // namespace mscl
} // extern "C"
#endif // __cplusplus
