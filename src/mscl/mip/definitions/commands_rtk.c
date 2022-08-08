
#include "commands_rtk.h"

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

void insert_mip_media_selector(struct mip_serializer* serializer, const enum mip_media_selector self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_media_selector(struct mip_serializer* serializer, enum mip_media_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_led_action(struct mip_serializer* serializer, const enum mip_led_action self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_led_action(struct mip_serializer* serializer, enum mip_led_action* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, const struct mip_rtk_connected_device_type_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_mip_rtk_connected_device_type_command_type(serializer, self->devType);
}

void extract_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, struct mip_rtk_connected_device_type_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_mip_rtk_connected_device_type_command_type(serializer, &self->devType);
}

void insert_mip_rtk_service_status_command(struct mip_serializer* serializer, const struct mip_rtk_service_status_command* self)
{
    insert_u32(serializer, self->reserved1);
    insert_u32(serializer, self->reserved2);
}

void extract_mip_rtk_service_status_command(struct mip_serializer* serializer, struct mip_rtk_service_status_command* self)
{
    extract_u32(serializer, &self->reserved1);
    extract_u32(serializer, &self->reserved2);
}

void insert_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, const struct mip_rtk_prod_erase_storage_command* self)
{
    insert_mip_media_selector(serializer, self->media);
}

void extract_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, struct mip_rtk_prod_erase_storage_command* self)
{
    extract_mip_media_selector(serializer, &self->media);
}

void insert_mip_rtk_led_control_command(struct mip_serializer* serializer, const struct mip_rtk_led_control_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_u8(serializer, self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(serializer, self->altColor[i]);
    insert_mip_led_action(serializer, self->act);
    insert_u32(serializer, self->period);
}

void extract_mip_rtk_led_control_command(struct mip_serializer* serializer, struct mip_rtk_led_control_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_u8(serializer, &self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        extract_u8(serializer, &self->altColor[i]);
    extract_mip_led_action(serializer, &self->act);
    extract_u32(serializer, &self->period);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

