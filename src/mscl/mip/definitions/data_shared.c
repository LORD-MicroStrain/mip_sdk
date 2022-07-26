
#include "data_shared.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

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
size_t insert_mip_shared_event_source_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_event_source_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->trigger_id);
    
    return offset;
}

size_t extract_mip_shared_event_source_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_event_source_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger_id);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_ticks_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_ticks_data* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->ticks);
    
    return offset;
}

size_t extract_mip_shared_ticks_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_ticks_data* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->ticks);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_delta_ticks_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_delta_ticks_data* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->ticks);
    
    return offset;
}

size_t extract_mip_shared_delta_ticks_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_delta_ticks_data* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->ticks);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_gps_timestamp_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_shared_gps_timestamp_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_shared_gps_timestamp_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_shared_gps_timestamp_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_shared_gps_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_gps_timestamp_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_mip_shared_gps_timestamp_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_shared_gps_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_gps_timestamp_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_mip_shared_gps_timestamp_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_delta_time_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_delta_time_data* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->seconds);
    
    return offset;
}

size_t extract_mip_shared_delta_time_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_delta_time_data* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->seconds);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_reference_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_reference_timestamp_data* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->nanoseconds);
    
    return offset;
}

size_t extract_mip_shared_reference_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_reference_timestamp_data* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->nanoseconds);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_reference_time_delta_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_reference_time_delta_data* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->dt_nanos);
    
    return offset;
}

size_t extract_mip_shared_reference_time_delta_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_reference_time_delta_data* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->dt_nanos);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_external_timestamp_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_shared_external_timestamp_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_shared_external_timestamp_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_shared_external_timestamp_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_shared_external_timestamp_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_external_timestamp_data* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->nanoseconds);
    offset = insert_mip_shared_external_timestamp_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_shared_external_timestamp_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_external_timestamp_data* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->nanoseconds);
    offset = extract_mip_shared_external_timestamp_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_shared_external_time_delta_data_valid_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_shared_external_time_delta_data_valid_flags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_mip_shared_external_time_delta_data_valid_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_shared_external_time_delta_data_valid_flags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_shared_external_time_delta_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_shared_external_time_delta_data* self)
{
    offset = insert_u64(buffer, bufferSize, offset, self->dt_nanos);
    offset = insert_mip_shared_external_time_delta_data_valid_flags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_mip_shared_external_time_delta_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_shared_external_time_delta_data* self)
{
    offset = extract_u64(buffer, bufferSize, offset, &self->dt_nanos);
    offset = extract_mip_shared_external_time_delta_data_valid_flags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
