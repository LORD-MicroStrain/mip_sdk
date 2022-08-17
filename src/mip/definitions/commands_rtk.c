
#include "commands_rtk.h"

#include "../utils/serialization.h"
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

enum mip_cmd_result mip_rtk_get_status_flags(struct mip_interface* device, enum mip_rtk_get_status_flags_command_status_flags* flags_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_STATUS_FLAGS, NULL, 0, MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(flags_out);
        extract_mip_rtk_get_status_flags_command_status_flags(&deserializer, flags_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_rtk_get_imei(struct mip_interface* device, char* imei_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMEI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMEI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(imei_out);
        for(unsigned int i=0; i < 32; i++)
            extract_char(&deserializer, &imei_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_rtk_get_imsi(struct mip_interface* device, char* imsi_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(imsi_out);
        for(unsigned int i=0; i < 32; i++)
            extract_char(&deserializer, &imsi_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_rtk_get_iccid(struct mip_interface* device, char* iccid_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ICCID, NULL, 0, MIP_REPLY_DESC_RTK_GET_ICCID, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(iccid_out);
        for(unsigned int i=0; i < 32; i++)
            extract_char(&deserializer, &iccid_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, const struct mip_rtk_connected_device_type_command* self)
{
    insert_mip_rtk_connected_device_type_command_type(serializer, self->devType);
    
}
void extract_mip_rtk_connected_device_type_command(struct mip_serializer* serializer, struct mip_rtk_connected_device_type_command* self)
{
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

enum mip_cmd_result mip_rtk_write_connected_device_type(struct mip_interface* device, enum mip_rtk_connected_device_type_command_type dev_type)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_rtk_connected_device_type_command_type(&serializer, dev_type);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}
enum mip_cmd_result mip_rtk_read_connected_device_type(struct mip_interface* device, enum mip_rtk_connected_device_type_command_type* dev_type_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset, MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(dev_type_out);
        extract_mip_rtk_connected_device_type_command_type(&deserializer, dev_type_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_rtk_save_connected_device_type(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}
enum mip_cmd_result mip_rtk_load_connected_device_type(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}
enum mip_cmd_result mip_rtk_default_connected_device_type(struct mip_interface* device)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, serializer.offset);
}
enum mip_cmd_result mip_rtk_get_act_code(struct mip_interface* device, char* activation_code_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ACT_CODE, NULL, 0, MIP_REPLY_DESC_RTK_GET_ACT_CODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(activation_code_out);
        for(unsigned int i=0; i < 32; i++)
            extract_char(&deserializer, &activation_code_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_rtk_get_modem_firmware_version(struct mip_interface* device, char* modem_firmware_version_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, NULL, 0, MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(modem_firmware_version_out);
        for(unsigned int i=0; i < 32; i++)
            extract_char(&deserializer, &modem_firmware_version_out[i]);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
enum mip_cmd_result mip_rtk_get_rssi(struct mip_interface* device, bool* valid_out, int32_t* rssi_out, int32_t* signal_quality_out)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_RSSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_RSSI, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(valid_out);
        extract_bool(&deserializer, valid_out);
        
        assert(rssi_out);
        extract_s32(&deserializer, rssi_out);
        
        assert(signal_quality_out);
        extract_s32(&deserializer, signal_quality_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
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

enum mip_cmd_result mip_rtk_service_status(struct mip_interface* device, uint32_t reserved1, uint32_t reserved2, enum mip_rtk_service_status_command_service_flags* flags_out, uint32_t* recieved_bytes_out, uint32_t* last_bytes_out, uint64_t* last_bytes_time_out)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u32(&serializer, reserved1);
    
    insert_u32(&serializer, reserved2);
    
    assert(mip_serializer_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    enum mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_SERVICE_STATUS, buffer, serializer.offset, MIP_REPLY_DESC_RTK_SERVICE_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        struct mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(flags_out);
        extract_mip_rtk_service_status_command_service_flags(&deserializer, flags_out);
        
        assert(recieved_bytes_out);
        extract_u32(&deserializer, recieved_bytes_out);
        
        assert(last_bytes_out);
        extract_u32(&deserializer, last_bytes_out);
        
        assert(last_bytes_time_out);
        extract_u64(&deserializer, last_bytes_time_out);
        
        if( !mip_serializer_ok(&deserializer) )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, const struct mip_rtk_prod_erase_storage_command* self)
{
    insert_mip_media_selector(serializer, self->media);
    
}
void extract_mip_rtk_prod_erase_storage_command(struct mip_serializer* serializer, struct mip_rtk_prod_erase_storage_command* self)
{
    extract_mip_media_selector(serializer, &self->media);
    
}

enum mip_cmd_result mip_rtk_prod_erase_storage(struct mip_interface* device, enum mip_media_selector media)
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

enum mip_cmd_result mip_rtk_led_control(struct mip_interface* device, const uint8_t* primary_color, const uint8_t* alt_color, enum mip_led_action act, uint32_t period)
{
    struct mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(primary_color);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(&serializer, primary_color[i]);
    
    assert(alt_color);
    for(unsigned int i=0; i < 3; i++)
        insert_u8(&serializer, alt_color[i]);
    
    insert_mip_led_action(&serializer, act);
    
    insert_u32(&serializer, period);
    
    assert(mip_serializer_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_LED_CONTROL, buffer, serializer.offset);
}
enum mip_cmd_result mip_rtk_modem_hard_reset(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_MODEM_HARD_RESET, NULL, 0);
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

