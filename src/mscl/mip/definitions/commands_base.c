
#include "commands_base.h"

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

void insert_mip_base_device_info(struct mip_serializer* serializer, const struct mip_base_device_info* self)
{
    insert_u16(serializer, self->firmware_version);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert_char(serializer, self->device_options[i]);
}

void extract_mip_base_device_info(struct mip_serializer* serializer, struct mip_base_device_info* self)
{
    extract_u16(serializer, &self->firmware_version);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract_char(serializer, &self->device_options[i]);
}

void insert_mip_time_format(struct mip_serializer* serializer, const enum mip_time_format self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_time_format(struct mip_serializer* serializer, enum mip_time_format* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_commanded_test_bits_gq7(struct mip_serializer* serializer, const enum mip_commanded_test_bits_gq7 self)
{
    return insert_u32(serializer, (uint32_t)(self));
}
void extract_mip_commanded_test_bits_gq7(struct mip_serializer* serializer, enum mip_commanded_test_bits_gq7* self)
{
    uint32_t tmp = 0;
    extract_u32(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_base_comm_speed_command(struct mip_serializer* serializer, const struct mip_base_comm_speed_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->port);
    insert_u32(serializer, self->baud);
}

void extract_mip_base_comm_speed_command(struct mip_serializer* serializer, struct mip_base_comm_speed_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->port);
    extract_u32(serializer, &self->baud);
}

void insert_mip_base_gps_time_update_command(struct mip_serializer* serializer, const struct mip_base_gps_time_update_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_base_gps_time_update_command_field_id(serializer, self->field_id);
    insert_u32(serializer, self->value);
}

void extract_mip_base_gps_time_update_command(struct mip_serializer* serializer, struct mip_base_gps_time_update_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_base_gps_time_update_command_field_id(serializer, &self->field_id);
    extract_u32(serializer, &self->value);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

