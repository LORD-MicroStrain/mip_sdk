
#include "data_shared.h"

#include "microstrain/common/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct microstrain_serializer;
struct mip_field;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_shared_event_source_data(microstrain_serializer* serializer, const mip_shared_event_source_data* self)
{
    microstrain_insert_u8(serializer, self->trigger_id);
    
}
void extract_mip_shared_event_source_data(microstrain_serializer* serializer, mip_shared_event_source_data* self)
{
    microstrain_extract_u8(serializer, &self->trigger_id);
    
}
bool extract_mip_shared_event_source_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_event_source_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_event_source_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_ticks_data(microstrain_serializer* serializer, const mip_shared_ticks_data* self)
{
    microstrain_insert_u32(serializer, self->ticks);
    
}
void extract_mip_shared_ticks_data(microstrain_serializer* serializer, mip_shared_ticks_data* self)
{
    microstrain_extract_u32(serializer, &self->ticks);
    
}
bool extract_mip_shared_ticks_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_ticks_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_ticks_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_delta_ticks_data(microstrain_serializer* serializer, const mip_shared_delta_ticks_data* self)
{
    microstrain_insert_u32(serializer, self->ticks);
    
}
void extract_mip_shared_delta_ticks_data(microstrain_serializer* serializer, mip_shared_delta_ticks_data* self)
{
    microstrain_extract_u32(serializer, &self->ticks);
    
}
bool extract_mip_shared_delta_ticks_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_delta_ticks_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_delta_ticks_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_gps_timestamp_data(microstrain_serializer* serializer, const mip_shared_gps_timestamp_data* self)
{
    microstrain_insert_double(serializer, self->tow);

    microstrain_insert_u16(serializer, self->week_number);
    
    insert_mip_shared_gps_timestamp_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_shared_gps_timestamp_data(microstrain_serializer* serializer, mip_shared_gps_timestamp_data* self)
{
    microstrain_extract_double(serializer, &self->tow);

    microstrain_extract_u16(serializer, &self->week_number);
    
    extract_mip_shared_gps_timestamp_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_shared_gps_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_gps_timestamp_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_gps_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_gps_timestamp_data_valid_flags(struct microstrain_serializer* serializer, const mip_shared_gps_timestamp_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_shared_gps_timestamp_data_valid_flags(struct microstrain_serializer* serializer, mip_shared_gps_timestamp_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_shared_delta_time_data(microstrain_serializer* serializer, const mip_shared_delta_time_data* self)
{
    microstrain_insert_double(serializer, self->seconds);
    
}
void extract_mip_shared_delta_time_data(microstrain_serializer* serializer, mip_shared_delta_time_data* self)
{
    microstrain_extract_double(serializer, &self->seconds);
    
}
bool extract_mip_shared_delta_time_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_delta_time_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_delta_time_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_reference_timestamp_data(microstrain_serializer* serializer, const mip_shared_reference_timestamp_data* self)
{
    microstrain_insert_u64(serializer, self->nanoseconds);
    
}
void extract_mip_shared_reference_timestamp_data(microstrain_serializer* serializer, mip_shared_reference_timestamp_data* self)
{
    microstrain_extract_u64(serializer, &self->nanoseconds);
    
}
bool extract_mip_shared_reference_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_reference_timestamp_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_reference_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_reference_time_delta_data(microstrain_serializer* serializer, const mip_shared_reference_time_delta_data* self)
{
    microstrain_insert_u64(serializer, self->dt_nanos);
    
}
void extract_mip_shared_reference_time_delta_data(microstrain_serializer* serializer, mip_shared_reference_time_delta_data* self)
{
    microstrain_extract_u64(serializer, &self->dt_nanos);
    
}
bool extract_mip_shared_reference_time_delta_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_reference_time_delta_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_reference_time_delta_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_external_timestamp_data(microstrain_serializer* serializer, const mip_shared_external_timestamp_data* self)
{
    microstrain_insert_u64(serializer, self->nanoseconds);
    
    insert_mip_shared_external_timestamp_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_shared_external_timestamp_data(microstrain_serializer* serializer, mip_shared_external_timestamp_data* self)
{
    microstrain_extract_u64(serializer, &self->nanoseconds);
    
    extract_mip_shared_external_timestamp_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_shared_external_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_external_timestamp_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_external_timestamp_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_external_timestamp_data_valid_flags(struct microstrain_serializer* serializer, const mip_shared_external_timestamp_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_shared_external_timestamp_data_valid_flags(struct microstrain_serializer* serializer, mip_shared_external_timestamp_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_shared_external_time_delta_data(microstrain_serializer* serializer, const mip_shared_external_time_delta_data* self)
{
    microstrain_insert_u64(serializer, self->dt_nanos);
    
    insert_mip_shared_external_time_delta_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_shared_external_time_delta_data(microstrain_serializer* serializer, mip_shared_external_time_delta_data* self)
{
    microstrain_extract_u64(serializer, &self->dt_nanos);
    
    extract_mip_shared_external_time_delta_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_shared_external_time_delta_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_shared_external_time_delta_data* self = ptr;
    struct microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_shared_external_time_delta_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_shared_external_time_delta_data_valid_flags(struct microstrain_serializer* serializer, const mip_shared_external_time_delta_data_valid_flags self)
{
    microstrain_insert_u16(serializer, (uint16_t) (self));
}
void extract_mip_shared_external_time_delta_data_valid_flags(struct microstrain_serializer* serializer, mip_shared_external_time_delta_data_valid_flags* self)
{
    uint16_t tmp = 0;
    microstrain_extract_u16(serializer, &tmp);
    *self = tmp;
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

