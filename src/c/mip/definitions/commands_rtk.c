
#include "commands_rtk.h"

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

mip_cmd_result mip_rtk_get_status_flags(mip_interface* device, mip_rtk_get_status_flags_command_status_flags* flags_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_STATUS_FLAGS, NULL, 0, MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(flags_out);
        extract_mip_rtk_get_status_flags_command_status_flags(&deserializer, flags_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_rtk_get_imei(mip_interface* device, char* imei_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMEI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMEI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(imei_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_char(&deserializer, &imei_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_rtk_get_imsi(mip_interface* device, char* imsi_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(imsi_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_char(&deserializer, &imsi_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_rtk_get_iccid(mip_interface* device, char* iccid_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ICCID, NULL, 0, MIP_REPLY_DESC_RTK_GET_ICCID, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(iccid_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_char(&deserializer, &iccid_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_rtk_connected_device_type_command(microstrain_serializer* serializer, const mip_rtk_connected_device_type_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_rtk_connected_device_type_command_type(serializer, self->devType);
        
    }
}
void extract_mip_rtk_connected_device_type_command(microstrain_serializer* serializer, mip_rtk_connected_device_type_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_rtk_connected_device_type_command_type(serializer, &self->devType);
        
    }
}

void insert_mip_rtk_connected_device_type_response(microstrain_serializer* serializer, const mip_rtk_connected_device_type_response* self)
{
    insert_mip_rtk_connected_device_type_command_type(serializer, self->devType);
    
}
void extract_mip_rtk_connected_device_type_response(microstrain_serializer* serializer, mip_rtk_connected_device_type_response* self)
{
    extract_mip_rtk_connected_device_type_command_type(serializer, &self->devType);
    
}

mip_cmd_result mip_rtk_write_connected_device_type(mip_interface* device, mip_rtk_connected_device_type_command_type dev_type)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_rtk_connected_device_type_command_type(&serializer, dev_type);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_rtk_read_connected_device_type(mip_interface* device, mip_rtk_connected_device_type_command_type* dev_type_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(dev_type_out);
        extract_mip_rtk_connected_device_type_command_type(&deserializer, dev_type_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_rtk_save_connected_device_type(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_rtk_load_connected_device_type(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_rtk_default_connected_device_type(mip_interface* device)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_rtk_get_act_code(mip_interface* device, char* activation_code_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ACT_CODE, NULL, 0, MIP_REPLY_DESC_RTK_GET_ACT_CODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(activation_code_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_char(&deserializer, &activation_code_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_rtk_get_modem_firmware_version(mip_interface* device, char* modem_firmware_version_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, NULL, 0, MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(modem_firmware_version_out);
        for(unsigned int i=0; i < 32; i++)
            microstrain_extract_char(&deserializer, &modem_firmware_version_out[i]);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_rtk_get_rssi(mip_interface* device, bool* valid_out, int32_t* rssi_out, int32_t* signal_quality_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_RSSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_RSSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(valid_out);
        microstrain_extract_bool(&deserializer, valid_out);
        
        assert(rssi_out);
        microstrain_extract_s32(&deserializer, rssi_out);
        
        assert(signal_quality_out);
        microstrain_extract_s32(&deserializer, signal_quality_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_rtk_service_status_command(microstrain_serializer* serializer, const mip_rtk_service_status_command* self)
{
    microstrain_insert_u32(serializer, self->reserved1);
    
    microstrain_insert_u32(serializer, self->reserved2);
    
}
void extract_mip_rtk_service_status_command(microstrain_serializer* serializer, mip_rtk_service_status_command* self)
{
    microstrain_extract_u32(serializer, &self->reserved1);
    
    microstrain_extract_u32(serializer, &self->reserved2);
    
}

void insert_mip_rtk_service_status_response(microstrain_serializer* serializer, const mip_rtk_service_status_response* self)
{
    insert_mip_rtk_service_status_command_service_flags(serializer, self->flags);
    
    microstrain_insert_u32(serializer, self->receivedBytes);
    
    microstrain_insert_u32(serializer, self->lastBytes);
    
    microstrain_insert_u64(serializer, self->lastBytesTime);
    
}
void extract_mip_rtk_service_status_response(microstrain_serializer* serializer, mip_rtk_service_status_response* self)
{
    extract_mip_rtk_service_status_command_service_flags(serializer, &self->flags);
    
    microstrain_extract_u32(serializer, &self->receivedBytes);
    
    microstrain_extract_u32(serializer, &self->lastBytes);
    
    microstrain_extract_u64(serializer, &self->lastBytesTime);
    
}

mip_cmd_result mip_rtk_service_status(mip_interface* device, uint32_t reserved1, uint32_t reserved2, mip_rtk_service_status_command_service_flags* flags_out, uint32_t* received_bytes_out, uint32_t* last_bytes_out, uint64_t* last_bytes_time_out)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    microstrain_insert_u32(&serializer, reserved1);
    
    microstrain_insert_u32(&serializer, reserved2);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_SERVICE_STATUS, buffer, (uint8_t)microstrain_serializer_length(&serializer), MIP_REPLY_DESC_RTK_SERVICE_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        microstrain_serializer deserializer;
        microstrain_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(flags_out);
        extract_mip_rtk_service_status_command_service_flags(&deserializer, flags_out);
        
        assert(received_bytes_out);
        microstrain_extract_u32(&deserializer, received_bytes_out);
        
        assert(last_bytes_out);
        microstrain_extract_u32(&deserializer, last_bytes_out);
        
        assert(last_bytes_time_out);
        microstrain_extract_u64(&deserializer, last_bytes_time_out);
        
        if( microstrain_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_rtk_prod_erase_storage_command(microstrain_serializer* serializer, const mip_rtk_prod_erase_storage_command* self)
{
    insert_mip_media_selector(serializer, self->media);
    
}
void extract_mip_rtk_prod_erase_storage_command(microstrain_serializer* serializer, mip_rtk_prod_erase_storage_command* self)
{
    extract_mip_media_selector(serializer, &self->media);
    
}

mip_cmd_result mip_rtk_prod_erase_storage(mip_interface* device, mip_media_selector media)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_media_selector(&serializer, media);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
void insert_mip_rtk_led_control_command(microstrain_serializer* serializer, const mip_rtk_led_control_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_u8(serializer, self->primaryColor[i]);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_u8(serializer, self->altColor[i]);
    
    insert_mip_led_action(serializer, self->act);
    
    microstrain_insert_u32(serializer, self->period);
    
}
void extract_mip_rtk_led_control_command(microstrain_serializer* serializer, mip_rtk_led_control_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_u8(serializer, &self->primaryColor[i]);
    
    for(unsigned int i=0; i < 3; i++)
        microstrain_extract_u8(serializer, &self->altColor[i]);
    
    extract_mip_led_action(serializer, &self->act);
    
    microstrain_extract_u32(serializer, &self->period);
    
}

mip_cmd_result mip_rtk_led_control(mip_interface* device, const uint8_t* primary_color, const uint8_t* alt_color, mip_led_action act, uint32_t period)
{
    microstrain_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(primary_color);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_u8(&serializer, primary_color[i]);
    
    assert(alt_color);
    for(unsigned int i=0; i < 3; i++)
        microstrain_insert_u8(&serializer, alt_color[i]);
    
    insert_mip_led_action(&serializer, act);
    
    microstrain_insert_u32(&serializer, period);
    
    assert(microstrain_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_LED_CONTROL, buffer, (uint8_t)microstrain_serializer_length(&serializer));
}
mip_cmd_result mip_rtk_modem_hard_reset(mip_interface* device)
{
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_MODEM_HARD_RESET, NULL, 0);
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus

