
#include "descriptors.h"

#include "../../utils/serialization.h"

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the descriptor set represents some kind of data.
///
///@param descriptorSet
///
///@returns true if the descriptor set represents data.
///@returns false otherwise.
///
bool isDataDescriptorSet(uint8_t descriptorSet)
{
    return (descriptorSet >= 0x80) && (descriptorSet < 0xF0);
}


size_t insert_MipFunctionSelector(uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFunctionSelector self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}

size_t extract_MipFunctionSelector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFunctionSelector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipDescriptorRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipDescriptorRate* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->descriptor);
    offset = insert_u16(buffer, bufferSize, offset, self->decimation);
    return offset;
}

size_t extract_MipDescriptorRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipDescriptorRate* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->descriptor);
    offset = extract_u16(buffer, bufferSize, offset, &self->decimation);
    return offset;
}


#ifdef __cplusplus
} // namespace mscl
} // extern "C"
#endif // __cplusplus
