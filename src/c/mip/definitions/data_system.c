
#include "data_system.h"

#include <mip/mip_serialization.h>
#include <mip/mip_interface.h>

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_system_built_in_test_data(microstrain_serializer* serializer, const mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        microstrain_insert_u8(serializer, self->result[i]);
    
}
void extract_mip_system_built_in_test_data(microstrain_serializer* serializer, mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        microstrain_extract_u8(serializer, &self->result[i]);
    
}
bool extract_mip_system_built_in_test_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_system_built_in_test_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_system_built_in_test_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_system_time_sync_status_data(microstrain_serializer* serializer, const mip_system_time_sync_status_data* self)
{
    microstrain_insert_bool(serializer, self->time_sync);
    
    microstrain_insert_u8(serializer, self->last_pps_rcvd);
    
}
void extract_mip_system_time_sync_status_data(microstrain_serializer* serializer, mip_system_time_sync_status_data* self)
{
    microstrain_extract_bool(serializer, &self->time_sync);
    
    microstrain_extract_u8(serializer, &self->last_pps_rcvd);
    
}
bool extract_mip_system_time_sync_status_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_system_time_sync_status_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_system_time_sync_status_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_system_gpio_state_data(microstrain_serializer* serializer, const mip_system_gpio_state_data* self)
{
    microstrain_insert_u8(serializer, self->states);
    
}
void extract_mip_system_gpio_state_data(microstrain_serializer* serializer, mip_system_gpio_state_data* self)
{
    microstrain_extract_u8(serializer, &self->states);
    
}
bool extract_mip_system_gpio_state_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_system_gpio_state_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_system_gpio_state_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}

void insert_mip_system_gpio_analog_value_data(microstrain_serializer* serializer, const mip_system_gpio_analog_value_data* self)
{
    microstrain_insert_u8(serializer, self->gpio_id);
    
    microstrain_insert_float(serializer, self->value);
    
}
void extract_mip_system_gpio_analog_value_data(microstrain_serializer* serializer, mip_system_gpio_analog_value_data* self)
{
    microstrain_extract_u8(serializer, &self->gpio_id);
    
    microstrain_extract_float(serializer, &self->value);
    
}
bool extract_mip_system_gpio_analog_value_data_from_field(const mip_field_view* field, void* ptr)
{
    assert(ptr);
    mip_system_gpio_analog_value_data* self = ptr;
    microstrain_serializer serializer;
    microstrain_serializer_init_from_field(&serializer, field);
    extract_mip_system_gpio_analog_value_data(&serializer, self);
    return microstrain_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

