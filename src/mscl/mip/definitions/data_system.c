
#include "data_system.h"

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
size_t insert_mip_system_built_in_test_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->result[i]);
    
    return offset;
}

size_t extract_mip_system_built_in_test_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->result[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_system_time_sync_status_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_time_sync_status_data* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->time_sync);
    offset = insert_u8(buffer, bufferSize, offset, self->last_pps_rcvd);
    
    return offset;
}

size_t extract_mip_system_time_sync_status_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_time_sync_status_data* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->time_sync);
    offset = extract_u8(buffer, bufferSize, offset, &self->last_pps_rcvd);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_system_gpio_state_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_gpio_state_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->states);
    
    return offset;
}

size_t extract_mip_system_gpio_state_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_gpio_state_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->states);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_system_gpio_analog_value_data(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_system_gpio_analog_value_data* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->gpio_id);
    offset = insert_float(buffer, bufferSize, offset, self->value);
    
    return offset;
}

size_t extract_mip_system_gpio_analog_value_data(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_system_gpio_analog_value_data* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->gpio_id);
    offset = extract_float(buffer, bufferSize, offset, &self->value);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
