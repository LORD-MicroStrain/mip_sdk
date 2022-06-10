
#include "commands_base.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

size_t insert_Mipbase_device_info(uint8_t* buffer, size_t bufferSize, size_t offset, const struct Mipbase_device_info* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->firmware_version);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->model_name[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->model_number[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->serial_number[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->lot_number[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_char(buffer, bufferSize, offset, self->device_options[i]);
    
    return offset;
}

size_t extract_Mipbase_device_info(const uint8_t* buffer, size_t bufferSize, size_t offset, struct Mipbase_device_info* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->firmware_version);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->model_name[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->model_number[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->serial_number[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->lot_number[i]);
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->device_options[i]);
    
    return offset;
}


size_t insert_MipTimeFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipTimeFormat self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipTimeFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipTimeFormat* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCommandedTestBits_Gq7(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCommandedTestBits_Gq7 self)
{
    return insert_u32(buffer, bufferSize, offset, self);
}
size_t extract_MipCommandedTestBits_Gq7(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCommandedTestBits_Gq7* self)
{
    uint32_t tmp;
    offset = extract_u32(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}



////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_Ping(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_Ping* self)
{
    return offset;
}

size_t extract_MipCmd_Base_Ping(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_Ping* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_SetIdle(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_SetIdle* self)
{
    return offset;
}

size_t extract_MipCmd_Base_SetIdle(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_SetIdle* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GetDeviceInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceInfo* self)
{
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceInfo* self)
{
    return offset;
}


size_t insert_MipCmd_Base_GetDeviceInfo_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceInfo_Response* self)
{
    offset = insert_Mipbase_device_info(buffer, bufferSize, offset, &self->device_info);
    
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceInfo_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceInfo_Response* self)
{
    offset = extract_Mipbase_device_info(buffer, bufferSize, offset, &self->device_info);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GetDeviceDescriptors(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceDescriptors* self)
{
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceDescriptors(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceDescriptors* self)
{
    return offset;
}


size_t insert_MipCmd_Base_GetDeviceDescriptors_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetDeviceDescriptors_Response* self)
{
    
    assert(self->descriptors_count <= 253);
    for(unsigned int i=0; i < self->descriptors_count; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->descriptors[i]);
    offset = insert_u8(buffer, bufferSize, offset, self->descriptors_count);
    
    return offset;
}

size_t extract_MipCmd_Base_GetDeviceDescriptors_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetDeviceDescriptors_Response* self)
{
    
    assert(self->descriptors_count <= 253);
    for(unsigned int i=0; i < self->descriptors_count; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->descriptors[i]);
    offset = extract_u8(buffer, bufferSize, offset, &self->descriptors_count);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_BuiltInTest(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_BuiltInTest* self)
{
    return offset;
}

size_t extract_MipCmd_Base_BuiltInTest(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_BuiltInTest* self)
{
    return offset;
}


size_t insert_MipCmd_Base_BuiltInTest_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_BuiltInTest_Response* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->result);
    
    return offset;
}

size_t extract_MipCmd_Base_BuiltInTest_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_BuiltInTest_Response* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->result);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_Resume(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_Resume* self)
{
    return offset;
}

size_t extract_MipCmd_Base_Resume(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_Resume* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GetExtendedDescriptors(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetExtendedDescriptors* self)
{
    return offset;
}

size_t extract_MipCmd_Base_GetExtendedDescriptors(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetExtendedDescriptors* self)
{
    return offset;
}


size_t insert_MipCmd_Base_GetExtendedDescriptors_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GetExtendedDescriptors_Response* self)
{
    
    assert(self->descriptors_count <= 253);
    for(unsigned int i=0; i < self->descriptors_count; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->descriptors[i]);
    offset = insert_u8(buffer, bufferSize, offset, self->descriptors_count);
    
    return offset;
}

size_t extract_MipCmd_Base_GetExtendedDescriptors_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GetExtendedDescriptors_Response* self)
{
    
    assert(self->descriptors_count <= 253);
    for(unsigned int i=0; i < self->descriptors_count; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->descriptors[i]);
    offset = extract_u8(buffer, bufferSize, offset, &self->descriptors_count);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_ContinuousBit(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_ContinuousBit* self)
{
    return offset;
}

size_t extract_MipCmd_Base_ContinuousBit(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_ContinuousBit* self)
{
    return offset;
}


size_t insert_MipCmd_Base_ContinuousBit_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_ContinuousBit_Response* self)
{
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->result[i]);
    
    return offset;
}

size_t extract_MipCmd_Base_ContinuousBit_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_ContinuousBit_Response* self)
{
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->result[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_CommSpeed(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_CommSpeed* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->port);
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_Base_CommSpeed(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_CommSpeed* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->port);
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


size_t insert_MipCmd_Base_CommSpeed_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_CommSpeed_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->port);
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_Base_CommSpeed_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_CommSpeed_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->port);
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_GpsTimeUpdate_Fieldid(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Base_GpsTimeUpdate_Fieldid self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_Base_GpsTimeUpdate_Fieldid(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Base_GpsTimeUpdate_Fieldid* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_Base_GpsTimeUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_GpsTimeUpdate* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_MipCmd_Base_GpsTimeUpdate_Fieldid(buffer, bufferSize, offset, self->field_id);
    offset = insert_u32(buffer, bufferSize, offset, self->value);
    
    return offset;
}

size_t extract_MipCmd_Base_GpsTimeUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_GpsTimeUpdate* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_MipCmd_Base_GpsTimeUpdate_Fieldid(buffer, bufferSize, offset, &self->field_id);
    offset = extract_u32(buffer, bufferSize, offset, &self->value);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Base_SoftReset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Base_SoftReset* self)
{
    return offset;
}

size_t extract_MipCmd_Base_SoftReset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Base_SoftReset* self)
{
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
