
#include "commands_rtk.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
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

void insert_mip_rtk_get_status_flags_command_status_flags_legacy(struct mip_serializer* serializer, const enum mip_rtk_get_status_flags_command_status_flags_legacy self)
{
    return insert_u32(serializer, (uint32_t)(self));
}
void extract_mip_rtk_get_status_flags_command_status_flags_legacy(struct mip_serializer* serializer, enum mip_rtk_get_status_flags_command_status_flags_legacy* self)
{
    uint32_t tmp = 0;
    extract_u32(serializer, &tmp);
    *self = tmp;
}

void insert_mip_rtk_get_status_flags_command_status_flags(struct mip_serializer* serializer, const enum mip_rtk_get_status_flags_command_status_flags self)
{
    return insert_u32(serializer, (uint32_t)(self));
}
void extract_mip_rtk_get_status_flags_command_status_flags(struct mip_serializer* serializer, enum mip_rtk_get_status_flags_command_status_flags* self)
{
    uint32_t tmp = 0;
    extract_u32(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_rtk_get_status_flags(struct mip_interface* device, enum mip_rtk_get_status_flags_command_status_flags* flags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_STATUS_FLAGS, NULL, 0, MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_rtk_get_status_flags_command_status_flags(&serializer, flags);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

mip_cmd_result mip_rtk_get_imei(struct mip_interface* device, char* IMEI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMEI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMEI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract_char(&serializer, &IMEI[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

mip_cmd_result mip_rtk_get_imsi(struct mip_interface* device, char* IMSI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMSI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract_char(&serializer, &IMSI[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

mip_cmd_result mip_rtk_get_iccid(struct mip_interface* device, char* ICCID)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ICCID, NULL, 0, MIP_REPLY_DESC_RTK_GET_ICCID, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract_char(&serializer, &ICCID[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

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

void insert_mip_rtk_connected_device_type_command_type(struct mip_serializer* serializer, const enum mip_rtk_connected_device_type_command_type self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_rtk_connected_device_type_command_type(struct mip_serializer* serializer, enum mip_rtk_connected_device_type_command_type* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_rtk_write_connected_device_type(struct mip_interface* device, enum mip_rtk_connected_device_type_command_type devType)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    insert_mip_rtk_connected_device_type_command_type(&serializer, devType);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

mip_cmd_result mip_rtk_read_connected_device_type(struct mip_interface* device, enum mip_rtk_connected_device_type_command_type* devType)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset, MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_rtk_connected_device_type_command_type(&serializer, devType);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

mip_cmd_result mip_rtk_save_connected_device_type(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

mip_cmd_result mip_rtk_load_connected_device_type(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

mip_cmd_result mip_rtk_default_connected_device_type(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}

mip_cmd_result mip_rtk_get_act_code(struct mip_interface* device, char* ActivationCode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ACT_CODE, NULL, 0, MIP_REPLY_DESC_RTK_GET_ACT_CODE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract_char(&serializer, &ActivationCode[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

mip_cmd_result mip_rtk_get_modem_firmware_version(struct mip_interface* device, char* ModemFirmwareVersion)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, NULL, 0, MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 32; i++)
            extract_char(&serializer, &ModemFirmwareVersion[i]);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Get the RSSI and connected/disconnected status of modem
/// 
/// @param[out] valid 
/// @param[out] rssi 
/// @param[out] signalQuality 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_rtk_get_rssi(struct mip_interface* device, bool* valid, int32_t* rssi, int32_t* signalQuality)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_RSSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_RSSI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_bool(&serializer, valid);
        extract_s32(&serializer, rssi);
        extract_s32(&serializer, signalQuality);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
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

void insert_mip_rtk_service_status_command_service_flags(struct mip_serializer* serializer, const enum mip_rtk_service_status_command_service_flags self)
{
    return insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_rtk_service_status_command_service_flags(struct mip_serializer* serializer, enum mip_rtk_service_status_command_service_flags* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

/// @brief The 3DMRTK will send this message to the server to indicate that the connection should remain open. The Server will respond with information and status.
/// 
/// @param reserved1 
/// @param reserved2 
/// @param[out] flags 
/// @param[out] recievedBytes 
/// @param[out] lastBytes 
/// @param[out] lastBytesTime 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_rtk_service_status(struct mip_interface* device, uint32_t reserved1, uint32_t reserved2, enum mip_rtk_service_status_command_service_flags* flags, uint32_t* recievedBytes, uint32_t* lastBytes, uint64_t* lastBytesTime)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u32(&serializer, reserved1);
    insert_u32(&serializer, reserved2);
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_SERVICE_STATUS, buffer, serializer.offset, MIP_REPLY_DESC_RTK_SERVICE_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        struct mip_serializer serializer;
        mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
        
        extract_mip_rtk_service_status_command_service_flags(&serializer, flags);
        extract_u32(&serializer, recievedBytes);
        extract_u32(&serializer, lastBytes);
        extract_u64(&serializer, lastBytesTime);
        
        if( !mip_serializer_ok(&serializer) )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, const struct mip_rtk_prod_erase_storage_command* self)
{
    insert_mip_media_selector(serializer, self->media);
}

void extract_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, struct mip_rtk_prod_erase_storage_command* self)
{
    extract_mip_media_selector(serializer, &self->media);
}

/// @brief This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
/// @param media 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_rtk_prod_erase_storage(struct mip_interface* device, enum mip_media_selector media)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_media_selector(&serializer, media);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE, buffer, serializer.offset);
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

/// @brief This command allows direct control of the LED on the 3DM RTK. This command is only available in calibration mode or Production Test Mode.
/// 
/// @param primaryColor 
/// @param altColor 
/// @param act 
/// @param period 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_rtk_led_control(struct mip_interface* device, const uint8_t* primaryColor, const uint8_t* altColor, enum mip_led_action act, uint32_t period)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    for(unsigned int i=0; i < 3; i++)
        insert_u8(&serializer, primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(&serializer, altColor[i]);
    insert_mip_led_action(&serializer, act);
    insert_u32(&serializer, period);
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_LED_CONTROL, buffer, serializer.offset);
}

/// @brief This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_rtk_modem_hard_reset(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_MODEM_HARD_RESET, NULL, 0);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

