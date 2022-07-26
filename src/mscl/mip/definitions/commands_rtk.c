
#include "commands_rtk.h"

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

size_t insert_mip_connected_device_type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_connected_device_type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_connected_device_type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_connected_device_type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_media_selector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_media_selector self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_media_selector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_media_selector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_mip_led_action(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_led_action self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_led_action(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_led_action* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_status_flags_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_status_flags_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_status_flags_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_status_flags_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_status_flags_command_status_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_rtk_get_status_flags_command_status_flags self)
{
    return insert_u32(buffer, bufferSize, offset, self);
}
size_t extract_mip_rtk_get_status_flags_command_status_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_rtk_get_status_flags_command_status_flags* self)
{
    uint32_t tmp;
    offset = extract_u32(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_rtk_get_status_flags_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_status_flags_response* self)
{
    offset = insert_mip_rtk_get_status_flags_command_status_flags(buffer, bufferSize, offset, self->flags);
    
    return offset;
}

size_t extract_mip_rtk_get_status_flags_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_status_flags_response* self)
{
    offset = extract_mip_rtk_get_status_flags_command_status_flags(buffer, bufferSize, offset, &self->flags);
    
    return offset;
}


mip_cmd_result mip_rtk_get_status_flags(struct mip_interface* device, enum mip_rtk_get_status_flags_command_status_flags* flags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_STATUS_FLAGS, NULL, 0, MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_rtk_get_status_flags_command_status_flags(buffer, sizeof(buffer), responseUsed, flags);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_imei_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imei_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_imei_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imei_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_imei_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imei_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->IMEI[i]);
    
    return offset;
}

size_t extract_mip_rtk_get_imei_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imei_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->IMEI[i]);
    
    return offset;
}


mip_cmd_result mip_rtk_get_imei(struct mip_interface* device, char* IMEI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMEI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMEI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 32; i++)
            responseUsed = extract_char(buffer, sizeof(buffer), responseUsed, &IMEI[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_imsi_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imsi_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_imsi_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imsi_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_imsi_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_imsi_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->IMSI[i]);
    
    return offset;
}

size_t extract_mip_rtk_get_imsi_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_imsi_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->IMSI[i]);
    
    return offset;
}


mip_cmd_result mip_rtk_get_imsi(struct mip_interface* device, char* IMSI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_IMSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMSI, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 32; i++)
            responseUsed = extract_char(buffer, sizeof(buffer), responseUsed, &IMSI[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_iccid_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_iccid_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_iccid_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_iccid_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_iccid_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_iccid_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ICCID[i]);
    
    return offset;
}

size_t extract_mip_rtk_get_iccid_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_iccid_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ICCID[i]);
    
    return offset;
}


mip_cmd_result mip_rtk_get_iccid(struct mip_interface* device, char* ICCID)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ICCID, NULL, 0, MIP_REPLY_DESC_RTK_GET_ICCID, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 32; i++)
            responseUsed = extract_char(buffer, sizeof(buffer), responseUsed, &ICCID[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_connected_device_type_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_connected_device_type_command* self)
{
    offset = insert_mip_connected_device_type(buffer, bufferSize, offset, self->devType);
    
    return offset;
}

size_t extract_mip_rtk_connected_device_type_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_connected_device_type_command* self)
{
    offset = extract_mip_connected_device_type(buffer, bufferSize, offset, &self->devType);
    
    return offset;
}


size_t insert_mip_rtk_connected_device_type_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_connected_device_type_response* self)
{
    offset = insert_mip_connected_device_type(buffer, bufferSize, offset, self->devType);
    
    return offset;
}

size_t extract_mip_rtk_connected_device_type_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_connected_device_type_response* self)
{
    offset = extract_mip_connected_device_type(buffer, bufferSize, offset, &self->devType);
    
    return offset;
}


mip_cmd_result write_mip_rtk_connected_device_type(struct mip_interface* device, enum mip_connected_device_type devType)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_WRITE);
    cmdUsed = insert_mip_connected_device_type(buffer, sizeof(buffer), cmdUsed, devType);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

mip_cmd_result read_mip_rtk_connected_device_type(struct mip_interface* device, enum mip_connected_device_type* devType)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_READ);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed, MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_connected_device_type(buffer, sizeof(buffer), responseUsed, devType);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

mip_cmd_result save_mip_rtk_connected_device_type(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_SAVE);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

mip_cmd_result load_mip_rtk_connected_device_type(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_LOAD);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

mip_cmd_result default_mip_rtk_connected_device_type(struct mip_interface* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_function_selector(buffer, sizeof(buffer), cmdUsed, MIP_FUNCTION_RESET);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_act_code_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_act_code_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_act_code_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_act_code_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_act_code_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_act_code_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ActivationCode[i]);
    
    return offset;
}

size_t extract_mip_rtk_get_act_code_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_act_code_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ActivationCode[i]);
    
    return offset;
}


mip_cmd_result mip_rtk_get_act_code(struct mip_interface* device, char* ActivationCode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_ACT_CODE, NULL, 0, MIP_REPLY_DESC_RTK_GET_ACT_CODE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 32; i++)
            responseUsed = extract_char(buffer, sizeof(buffer), responseUsed, &ActivationCode[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_modem_firmware_version_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_modem_firmware_version_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_modem_firmware_version_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_modem_firmware_version_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_modem_firmware_version_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_modem_firmware_version_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ModemFirmwareVersion[i]);
    
    return offset;
}

size_t extract_mip_rtk_get_modem_firmware_version_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_modem_firmware_version_response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ModemFirmwareVersion[i]);
    
    return offset;
}


mip_cmd_result mip_rtk_get_modem_firmware_version(struct mip_interface* device, char* ModemFirmwareVersion)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, NULL, 0, MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 32; i++)
            responseUsed = extract_char(buffer, sizeof(buffer), responseUsed, &ModemFirmwareVersion[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_get_rssi_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_rssi_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_get_rssi_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_rssi_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_mip_rtk_get_rssi_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_get_rssi_response* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->valid);
    offset = insert_s32(buffer, bufferSize, offset, self->rssi);
    offset = insert_s32(buffer, bufferSize, offset, self->signalQuality);
    
    return offset;
}

size_t extract_mip_rtk_get_rssi_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_get_rssi_response* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->valid);
    offset = extract_s32(buffer, bufferSize, offset, &self->rssi);
    offset = extract_s32(buffer, bufferSize, offset, &self->signalQuality);
    
    return offset;
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
        size_t responseUsed = 0;
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, valid);
        responseUsed = extract_s32(buffer, sizeof(buffer), responseUsed, rssi);
        responseUsed = extract_s32(buffer, sizeof(buffer), responseUsed, signalQuality);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_service_status_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_service_status_command* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->reserved1);
    offset = insert_u32(buffer, bufferSize, offset, self->reserved2);
    
    return offset;
}

size_t extract_mip_rtk_service_status_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_service_status_command* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->reserved1);
    offset = extract_u32(buffer, bufferSize, offset, &self->reserved2);
    
    return offset;
}


size_t insert_mip_rtk_service_status_command_service_flags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum mip_rtk_service_status_command_service_flags self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_mip_rtk_service_status_command_service_flags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum mip_rtk_service_status_command_service_flags* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_mip_rtk_service_status_response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_service_status_response* self)
{
    offset = insert_mip_rtk_service_status_command_service_flags(buffer, bufferSize, offset, self->flags);
    offset = insert_u32(buffer, bufferSize, offset, self->recievedBytes);
    offset = insert_u32(buffer, bufferSize, offset, self->lastBytes);
    offset = insert_u64(buffer, bufferSize, offset, self->lastBytesTime);
    
    return offset;
}

size_t extract_mip_rtk_service_status_response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_service_status_response* self)
{
    offset = extract_mip_rtk_service_status_command_service_flags(buffer, bufferSize, offset, &self->flags);
    offset = extract_u32(buffer, bufferSize, offset, &self->recievedBytes);
    offset = extract_u32(buffer, bufferSize, offset, &self->lastBytes);
    offset = extract_u64(buffer, bufferSize, offset, &self->lastBytesTime);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, reserved1);
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, reserved2);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_SERVICE_STATUS, buffer, cmdUsed, MIP_REPLY_DESC_RTK_SERVICE_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_mip_rtk_service_status_command_service_flags(buffer, sizeof(buffer), responseUsed, flags);
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, recievedBytes);
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, lastBytes);
        responseUsed = extract_u64(buffer, sizeof(buffer), responseUsed, lastBytesTime);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_prod_erase_storage_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_prod_erase_storage_command* self)
{
    offset = insert_mip_media_selector(buffer, bufferSize, offset, self->media);
    
    return offset;
}

size_t extract_mip_rtk_prod_erase_storage_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_prod_erase_storage_command* self)
{
    offset = extract_mip_media_selector(buffer, bufferSize, offset, &self->media);
    
    return offset;
}


/// @brief This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
/// @param media 
/// 
/// @returns mip_cmd_result
/// 
mip_cmd_result mip_rtk_prod_erase_storage(struct mip_interface* device, enum mip_media_selector media)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_mip_media_selector(buffer, sizeof(buffer), cmdUsed, media);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_led_control_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_led_control_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->altColor[i]);
    offset = insert_mip_led_action(buffer, bufferSize, offset, self->act);
    offset = insert_u32(buffer, bufferSize, offset, self->period);
    
    return offset;
}

size_t extract_mip_rtk_led_control_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_led_control_command* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->altColor[i]);
    offset = extract_mip_led_action(buffer, bufferSize, offset, &self->act);
    offset = extract_u32(buffer, bufferSize, offset, &self->period);
    
    return offset;
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
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, altColor[i]);
    cmdUsed = insert_mip_led_action(buffer, sizeof(buffer), cmdUsed, act);
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, period);
    assert(cmdUsed <= sizeof(buffer));
    
    return mip_interface_run_command(device, MIP_RTK_CMD_DESC_SET, MIP_CMD_DESC_LED_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_mip_rtk_modem_hard_reset_command(uint8_t* buffer, size_t bufferSize, size_t offset, const struct mip_rtk_modem_hard_reset_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_mip_rtk_modem_hard_reset_command(const uint8_t* buffer, size_t bufferSize, size_t offset, struct mip_rtk_modem_hard_reset_command* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
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
} // extern "C"
} // namespace mscl
#endif // __cplusplus
