
#include "data_shared.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_shared_event_source_data(struct mip_serializer* serializer, const struct mip_shared_event_source_data* self)
{
    insert_u8(serializer, self->trigger_id);
}

void extract_mip_shared_event_source_data(struct mip_serializer* serializer, struct mip_shared_event_source_data* self)
{
    extract_u8(serializer, &self->trigger_id);
}

void insert_mip_shared_ticks_data(struct mip_serializer* serializer, const struct mip_shared_ticks_data* self)
{
    insert_u32(serializer, self->ticks);
}

void extract_mip_shared_ticks_data(struct mip_serializer* serializer, struct mip_shared_ticks_data* self)
{
    extract_u32(serializer, &self->ticks);
}

void insert_mip_shared_delta_ticks_data(struct mip_serializer* serializer, const struct mip_shared_delta_ticks_data* self)
{
    insert_u32(serializer, self->ticks);
}

void extract_mip_shared_delta_ticks_data(struct mip_serializer* serializer, struct mip_shared_delta_ticks_data* self)
{
    extract_u32(serializer, &self->ticks);
}

void insert_mip_shared_gps_timestamp_data(struct mip_serializer* serializer, const struct mip_shared_gps_timestamp_data* self)
{
    insert_double(serializer, self->tow);
    insert_u16(serializer, self->week_number);
    insert_mip_shared_gps_timestamp_data_valid_flags(serializer, self->valid_flags);
}

void extract_mip_shared_gps_timestamp_data(struct mip_serializer* serializer, struct mip_shared_gps_timestamp_data* self)
{
    extract_double(serializer, &self->tow);
    extract_u16(serializer, &self->week_number);
    extract_mip_shared_gps_timestamp_data_valid_flags(serializer, &self->valid_flags);
}

void insert_mip_shared_delta_time_data(struct mip_serializer* serializer, const struct mip_shared_delta_time_data* self)
{
    insert_double(serializer, self->seconds);
}

void extract_mip_shared_delta_time_data(struct mip_serializer* serializer, struct mip_shared_delta_time_data* self)
{
    extract_double(serializer, &self->seconds);
}

void insert_mip_shared_reference_timestamp_data(struct mip_serializer* serializer, const struct mip_shared_reference_timestamp_data* self)
{
    insert_u64(serializer, self->nanoseconds);
}

void extract_mip_shared_reference_timestamp_data(struct mip_serializer* serializer, struct mip_shared_reference_timestamp_data* self)
{
    extract_u64(serializer, &self->nanoseconds);
}

void insert_mip_shared_reference_time_delta_data(struct mip_serializer* serializer, const struct mip_shared_reference_time_delta_data* self)
{
    insert_u64(serializer, self->dt_nanos);
}

void extract_mip_shared_reference_time_delta_data(struct mip_serializer* serializer, struct mip_shared_reference_time_delta_data* self)
{
    extract_u64(serializer, &self->dt_nanos);
}

void insert_mip_shared_external_timestamp_data(struct mip_serializer* serializer, const struct mip_shared_external_timestamp_data* self)
{
    insert_u64(serializer, self->nanoseconds);
    insert_mip_shared_external_timestamp_data_valid_flags(serializer, self->valid_flags);
}

void extract_mip_shared_external_timestamp_data(struct mip_serializer* serializer, struct mip_shared_external_timestamp_data* self)
{
    extract_u64(serializer, &self->nanoseconds);
    extract_mip_shared_external_timestamp_data_valid_flags(serializer, &self->valid_flags);
}

void insert_mip_shared_external_time_delta_data(struct mip_serializer* serializer, const struct mip_shared_external_time_delta_data* self)
{
    insert_u64(serializer, self->dt_nanos);
    insert_mip_shared_external_time_delta_data_valid_flags(serializer, self->valid_flags);
}

void extract_mip_shared_external_time_delta_data(struct mip_serializer* serializer, struct mip_shared_external_time_delta_data* self)
{
    extract_u64(serializer, &self->dt_nanos);
    extract_mip_shared_external_time_delta_data_valid_flags(serializer, &self->valid_flags);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

