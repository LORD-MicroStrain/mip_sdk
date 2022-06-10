
#include "data_shared.h"

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
size_t insert_MipData_Shared_EventSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_EventSource* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->trigger_id);
    
    return offset;
}

size_t extract_MipData_Shared_EventSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_EventSource* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger_id);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_Ticks(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_Ticks* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->ticks);
    
    return offset;
}

size_t extract_MipData_Shared_Ticks(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_Ticks* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->ticks);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_DeltaTicks(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_DeltaTicks* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->ticks);
    
    return offset;
}

size_t extract_MipData_Shared_DeltaTicks(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_DeltaTicks* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->ticks);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_GpsTimestamp_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Shared_GpsTimestamp_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Shared_GpsTimestamp_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Shared_GpsTimestamp_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Shared_GpsTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_GpsTimestamp* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipData_Shared_GpsTimestamp_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Shared_GpsTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_GpsTimestamp* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipData_Shared_GpsTimestamp_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_DeltaTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_DeltaTime* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->seconds);
    
    return offset;
}

size_t extract_MipData_Shared_DeltaTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_DeltaTime* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->seconds);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_ReferenceTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ReferenceTimestamp* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->nanoseconds);
    
    return offset;
}

size_t extract_MipData_Shared_ReferenceTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ReferenceTimestamp* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->nanoseconds);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_ReferenceTimeDelta(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ReferenceTimeDelta* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->dt_nanos);
    
    return offset;
}

size_t extract_MipData_Shared_ReferenceTimeDelta(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ReferenceTimeDelta* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->dt_nanos);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_ExternalTimestamp_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Shared_ExternalTimestamp_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Shared_ExternalTimestamp_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Shared_ExternalTimestamp_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Shared_ExternalTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ExternalTimestamp* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->nanoseconds);
    offset = insert_MipData_Shared_ExternalTimestamp_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Shared_ExternalTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ExternalTimestamp* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->nanoseconds);
    offset = extract_MipData_Shared_ExternalTimestamp_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Shared_ExternalTimeDelta_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Shared_ExternalTimeDelta_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Shared_ExternalTimeDelta_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Shared_ExternalTimeDelta_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Shared_ExternalTimeDelta(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Shared_ExternalTimeDelta* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->dt_nanos);
    offset = insert_MipData_Shared_ExternalTimeDelta_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Shared_ExternalTimeDelta(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Shared_ExternalTimeDelta* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->dt_nanos);
    offset = extract_MipData_Shared_ExternalTimeDelta_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
