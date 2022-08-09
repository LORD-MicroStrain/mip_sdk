
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

void insert_mip_gnss_receiver_info_command_info(struct mip_serializer* serializer, const struct mip_gnss_receiver_info_command_info* self)
{
    insert_u8(serializer, self->receiver_id);
    insert_u8(serializer, self->mip_data_descriptor_set);
    for(unsigned int i=0; i < 32; i++)
        insert_char(serializer, self->description[i]);
}

void extract_mip_gnss_receiver_info_command_info(struct mip_serializer* serializer, struct mip_gnss_receiver_info_command_info* self)
{
    extract_u8(serializer, &self->receiver_id);
    extract_u8(serializer, &self->mip_data_descriptor_set);
    for(unsigned int i=0; i < 32; i++)
        extract_char(serializer, &self->description[i]);
}

/// @brief Return information about the GNSS receivers in the device.
/// 
/// @param[out] num_receivers Number of physical receivers in the device
/// @param[out] receiver_info 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_receiver_info(struct mip_interface* device, uint8_t* num_receivers, struct mip_gnss_receiver_info_command_info* receiver_info)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_LIST_RECEIVERS, NULL, 0, MIP_REPLY_DESC_GNSS_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_count(&serializer, num_receivers, *num_receivers);
        for(unsigned int i=0; i < *num_receivers; i++)
            extract_mip_gnss_receiver_info_command_info(&serializer, &receiver_info[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

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

/// @brief Configure the GNSS signals used by the device.
/// 
/// @param gps_enable Bitfield 0: Enable L1CA, 1: Enable L2C
/// @param glonass_enable Bitfield 0: Enable L1OF, 1: Enable L2OF
/// @param galileo_enable Bitfield 0: Enable E1,   1: Enable E5B
/// @param beidou_enable Bitfield 0: Enable B1,   1: Enable B2
/// @param reserved 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_write_signal_configuration(struct mip_interface* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, gps_enable);
    insert_u8(&serializer, glonass_enable);
    insert_u8(&serializer, galileo_enable);
    insert_u8(&serializer, beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        insert_u8(&serializer, reserved[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// @param[out] gps_enable Bitfield 0: Enable L1CA, 1: Enable L2C
/// @param[out] glonass_enable Bitfield 0: Enable L1OF, 1: Enable L2OF
/// @param[out] galileo_enable Bitfield 0: Enable E1,   1: Enable E5B
/// @param[out] beidou_enable Bitfield 0: Enable B1,   1: Enable B2
/// @param[out] reserved 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_read_signal_configuration(struct mip_interface* device, uint8_t* gps_enable, uint8_t* glonass_enable, uint8_t* galileo_enable, uint8_t* beidou_enable, uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, gps_enable);
        extract_u8(&serializer, glonass_enable);
        extract_u8(&serializer, galileo_enable);
        extract_u8(&serializer, beidou_enable);
        for(unsigned int i=0; i < 4; i++)
            extract_u8(&serializer, &reserved[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_save_signal_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_load_signal_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_default_signal_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, serializer.offset);
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

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// @param enable 0 - Disabled, 1- Enabled
/// @param reserved 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_write_rtk_dongle_configuration(struct mip_interface* device, uint8_t enable, const uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_u8(&serializer, enable);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(&serializer, reserved[i]);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// @param[out] enable 
/// @param[out] reserved 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_read_rtk_dongle_configuration(struct mip_interface* device, uint8_t* enable, uint8_t* reserved)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset, MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_u8(&serializer, enable);
        for(unsigned int i=0; i < 3; i++)
            extract_u8(&serializer, &reserved[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_save_rtk_dongle_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_load_rtk_dongle_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_default_rtk_dongle_configuration(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
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

/// @brief Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
/// @param receiver_id Receiver id: e.g. 1, 2, etc.
/// @param enable 0 - Disabled, 1- Enabled
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_gnss_receiver_safe_mode(struct mip_interface* device, uint8_t receiver_id, uint8_t enable)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, receiver_id);
    insert_u8(&serializer, enable);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_GNSS_CMD_DESC_SET, MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE, buffer, serializer.offset);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

