
#include "data_system.h"

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

void insert_mip_system_built_in_test_data(struct mip_serializer* serializer, const struct mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        insert_u8(serializer, self->result[i]);
}

void extract_mip_system_built_in_test_data(struct mip_serializer* serializer, struct mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        extract_u8(serializer, &self->result[i]);
}

void insert_mip_system_time_sync_status_data(struct mip_serializer* serializer, const struct mip_system_time_sync_status_data* self)
{
    insert_bool(serializer, self->time_sync);
    insert_u8(serializer, self->last_pps_rcvd);
}

void extract_mip_system_time_sync_status_data(struct mip_serializer* serializer, struct mip_system_time_sync_status_data* self)
{
    extract_bool(serializer, &self->time_sync);
    extract_u8(serializer, &self->last_pps_rcvd);
}

void insert_mip_system_gpio_state_data(struct mip_serializer* serializer, const struct mip_system_gpio_state_data* self)
{
    insert_u8(serializer, self->states);
}

void extract_mip_system_gpio_state_data(struct mip_serializer* serializer, struct mip_system_gpio_state_data* self)
{
    extract_u8(serializer, &self->states);
}

void insert_mip_system_gpio_analog_value_data(struct mip_serializer* serializer, const struct mip_system_gpio_analog_value_data* self)
{
    insert_u8(serializer, self->gpio_id);
    insert_float(serializer, self->value);
}

void extract_mip_system_gpio_analog_value_data(struct mip_serializer* serializer, struct mip_system_gpio_analog_value_data* self)
{
    extract_u8(serializer, &self->gpio_id);
    extract_float(serializer, &self->value);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

