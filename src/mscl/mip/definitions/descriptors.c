
#include "descriptors.h"

#include "../../utils/serialization.h"

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set represents some kind of data.
///
///@param descriptor_set
///
///@returns true if the descriptor set represents data.
///@returns false otherwise.
///
bool is_data_descriptor_set(uint8_t descriptor_set)
{
    return (descriptor_set >= 0x80) && (descriptor_set < 0xF0);
}


size_t insert_mip_function_selector(uint8_t* buffer, size_t buffer_size, size_t offset, enum mip_function_selector self)
{
    return insert_u8(buffer, buffer_size, offset, self);
}

size_t extract_mip_function_selector(const uint8_t* buffer, size_t buffer_size, size_t offset, enum mip_function_selector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, buffer_size, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_descriptor_rate(uint8_t* buffer, size_t buffer_size, size_t offset, const struct mip_descriptor_rate* self)
{
    offset = insert_u8(buffer, buffer_size, offset, self->descriptor);
    offset = insert_u16(buffer, buffer_size, offset, self->decimation);
    return offset;
}

size_t extract_mip_descriptor_rate(const uint8_t* buffer, size_t buffer_size, size_t offset, struct mip_descriptor_rate* self)
{
    offset = extract_u8(buffer, buffer_size, offset, &self->descriptor);
    offset = extract_u16(buffer, buffer_size, offset, &self->decimation);
    return offset;
}


#ifdef __cplusplus
} // namespace mscl
} // extern "C"
#endif // __cplusplus
