
#include "commands_gnss.h"

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

void insert_mip_gnss_signal_configuration_command(struct mip_serializer* serializer, const struct mip_gnss_signal_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->gps_enable);
    insert_u8(serializer, self->glonass_enable);
    insert_u8(serializer, self->galileo_enable);
    insert_u8(serializer, self->beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        insert_u8(serializer, self->reserved[i]);
}

void extract_mip_gnss_signal_configuration_command(struct mip_serializer* serializer, struct mip_gnss_signal_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->gps_enable);
    extract_u8(serializer, &self->glonass_enable);
    extract_u8(serializer, &self->galileo_enable);
    extract_u8(serializer, &self->beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        extract_u8(serializer, &self->reserved[i]);
}

void insert_mip_gnss_rtk_dongle_configuration_command(struct mip_serializer* serializer, const struct mip_gnss_rtk_dongle_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->enable);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(serializer, self->reserved[i]);
}

void extract_mip_gnss_rtk_dongle_configuration_command(struct mip_serializer* serializer, struct mip_gnss_rtk_dongle_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->enable);
    for(unsigned int i=0; i < 3; i++)
        extract_u8(serializer, &self->reserved[i]);
}

void insert_mip_gnss_receiver_safe_mode_command(struct mip_serializer* serializer, const struct mip_gnss_receiver_safe_mode_command* self)
{
    insert_u8(serializer, self->receiver_id);
    insert_u8(serializer, self->enable);
}

void extract_mip_gnss_receiver_safe_mode_command(struct mip_serializer* serializer, struct mip_gnss_receiver_safe_mode_command* self)
{
    extract_u8(serializer, &self->receiver_id);
    extract_u8(serializer, &self->enable);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

