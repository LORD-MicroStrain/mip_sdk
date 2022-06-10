
#include "data_system.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_System_BuiltInTest(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_BuiltInTest* self)
{
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->result[i]);
    
    return offset;
}

size_t extract_MipData_System_BuiltInTest(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_BuiltInTest* self)
{
    
    assert(16 <= 16);
    for(unsigned int i=0; i < 16; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->result[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_System_TimeSyncStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_TimeSyncStatus* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->time_sync);
    offset = insert_u8(buffer, bufferSize, offset, self->last_pps_rcvd);
    
    return offset;
}

size_t extract_MipData_System_TimeSyncStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_TimeSyncStatus* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->time_sync);
    offset = extract_u8(buffer, bufferSize, offset, &self->last_pps_rcvd);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_System_GpioState(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_GpioState* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->states);
    
    return offset;
}

size_t extract_MipData_System_GpioState(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_GpioState* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->states);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_System_GpioAnalogValue(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_System_GpioAnalogValue* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->gpio_id);
    offset = insert_float(buffer, bufferSize, offset, self->value);
    
    return offset;
}

size_t extract_MipData_System_GpioAnalogValue(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_System_GpioAnalogValue* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->gpio_id);
    offset = extract_float(buffer, bufferSize, offset, &self->value);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
