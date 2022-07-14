
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

size_t insert_MipConnectedDeviceType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipConnectedDeviceType self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipConnectedDeviceType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipConnectedDeviceType* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipMediaSelector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipMediaSelector self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipMediaSelector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipMediaSelector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipLedAction(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipLedAction self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipLedAction(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipLedAction* self)
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
size_t insert_MipCmd_Rtk_GetStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetStatusFlags* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetStatusFlags* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetStatusFlags_Statusflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Rtk_GetStatusFlags_Statusflags self)
{
    return insert_u32(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_Rtk_GetStatusFlags_Statusflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Rtk_GetStatusFlags_Statusflags* self)
{
    uint32_t tmp;
    offset = extract_u32(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_Rtk_GetStatusFlags_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetStatusFlags_Response* self)
{
    offset = insert_MipCmd_Rtk_GetStatusFlags_Statusflags(buffer, bufferSize, offset, self->flags);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetStatusFlags_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetStatusFlags_Response* self)
{
    offset = extract_MipCmd_Rtk_GetStatusFlags_Statusflags(buffer, bufferSize, offset, &self->flags);
    
    return offset;
}


MipCmdResult get_rtk_device_status_flags(struct MipInterfaceState* device, enum MipCmd_Rtk_GetStatusFlags_Statusflags* flags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_STATUS_FLAGS, NULL, 0, MIP_REPLY_DESC_RTK_GET_STATUS_FLAGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipCmd_Rtk_GetStatusFlags_Statusflags(buffer, sizeof(buffer), responseUsed, flags);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetImei(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImei* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImei(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImei* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetImei_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImei_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->IMEI[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImei_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImei_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->IMEI[i]);
    
    return offset;
}


MipCmdResult get_rtk_device_imei_international_mobile_equipment_identifier(struct MipInterfaceState* device, char* IMEI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_IMEI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMEI, buffer, &responseLength);
    
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
size_t insert_MipCmd_Rtk_GetImsi(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImsi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImsi(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImsi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetImsi_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImsi_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->IMSI[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImsi_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImsi_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->IMSI[i]);
    
    return offset;
}


MipCmdResult get_rtk_device_imsi_international_mobile_subscriber_identifier(struct MipInterfaceState* device, char* IMSI)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_IMSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_IMSI, buffer, &responseLength);
    
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
size_t insert_MipCmd_Rtk_GetIccid(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetIccid* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetIccid(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetIccid* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetIccid_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetIccid_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ICCID[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetIccid_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetIccid_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ICCID[i]);
    
    return offset;
}


MipCmdResult get_rtk_device_iccid_integrated_circuit_card_identification_sim_number(struct MipInterfaceState* device, char* ICCID)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_ICCID, NULL, 0, MIP_REPLY_DESC_RTK_GET_ICCID, buffer, &responseLength);
    
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
size_t insert_MipCmd_Rtk_ConnectedDeviceType(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ConnectedDeviceType* self)
{
    offset = insert_MipConnectedDeviceType(buffer, bufferSize, offset, self->devType);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ConnectedDeviceType(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ConnectedDeviceType* self)
{
    offset = extract_MipConnectedDeviceType(buffer, bufferSize, offset, &self->devType);
    
    return offset;
}


size_t insert_MipCmd_Rtk_ConnectedDeviceType_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ConnectedDeviceType_Response* self)
{
    offset = insert_MipConnectedDeviceType(buffer, bufferSize, offset, self->devType);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ConnectedDeviceType_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ConnectedDeviceType_Response* self)
{
    offset = extract_MipConnectedDeviceType(buffer, bufferSize, offset, &self->devType);
    
    return offset;
}


MipCmdResult write_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device, enum MipConnectedDeviceType devType)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_MipConnectedDeviceType(buffer, sizeof(buffer), cmdUsed, devType);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

MipCmdResult read_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device, enum MipConnectedDeviceType* devType)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed, MIP_REPLY_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipConnectedDeviceType(buffer, sizeof(buffer), responseUsed, devType);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

MipCmdResult save_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

MipCmdResult load_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

MipCmdResult default_configure_or_read_the_type_of_the_connected_device(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_CONNECTED_DEVICE_TYPE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetActCode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetActCode* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetActCode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetActCode* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetActCode_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetActCode_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ActivationCode[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetActCode_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetActCode_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ActivationCode[i]);
    
    return offset;
}


MipCmdResult get_rtk_device_activation_code(struct MipInterfaceState* device, char* ActivationCode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_ACT_CODE, NULL, 0, MIP_REPLY_DESC_RTK_GET_ACT_CODE, buffer, &responseLength);
    
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
size_t insert_MipCmd_Rtk_GetModemFirmwareVersion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetModemFirmwareVersion* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetModemFirmwareVersion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetModemFirmwareVersion* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetModemFirmwareVersion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetModemFirmwareVersion_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ModemFirmwareVersion[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetModemFirmwareVersion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetModemFirmwareVersion_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ModemFirmwareVersion[i]);
    
    return offset;
}


MipCmdResult get_rtk_devices_cell_modem_firmware_version_number(struct MipInterfaceState* device, char* ModemFirmwareVersion)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, NULL, 0, MIP_REPLY_DESC_RTK_GET_MODEM_FIRMWARE_VERSION, buffer, &responseLength);
    
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
size_t insert_MipCmd_Rtk_GetRssi(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetRssi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetRssi(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetRssi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetRssi_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetRssi_Response* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->valid);
    offset = insert_s32(buffer, bufferSize, offset, self->rssi);
    offset = insert_s32(buffer, bufferSize, offset, self->signalQuality);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetRssi_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetRssi_Response* self)
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
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_rtk_get_rssi(struct MipInterfaceState* device, bool* valid, int32_t* rssi, int32_t* signalQuality)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_GET_RSSI, NULL, 0, MIP_REPLY_DESC_RTK_GET_RSSI, buffer, &responseLength);
    
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
size_t insert_MipCmd_Rtk_ServiceStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ServiceStatus* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->reserved1);
    offset = insert_u32(buffer, bufferSize, offset, self->reserved2);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ServiceStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ServiceStatus* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->reserved1);
    offset = extract_u32(buffer, bufferSize, offset, &self->reserved2);
    
    return offset;
}


size_t insert_MipCmd_Rtk_ServiceStatus_Serviceflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Rtk_ServiceStatus_Serviceflags self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_Rtk_ServiceStatus_Serviceflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Rtk_ServiceStatus_Serviceflags* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_Rtk_ServiceStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ServiceStatus_Response* self)
{
    offset = insert_MipCmd_Rtk_ServiceStatus_Serviceflags(buffer, bufferSize, offset, self->flags);
    offset = insert_u32(buffer, bufferSize, offset, self->recievedBytes);
    offset = insert_u32(buffer, bufferSize, offset, self->lastBytes);
    offset = insert_u64(buffer, bufferSize, offset, self->lastBytesTime);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ServiceStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ServiceStatus_Response* self)
{
    offset = extract_MipCmd_Rtk_ServiceStatus_Serviceflags(buffer, bufferSize, offset, &self->flags);
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
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_rtk_service_status(struct MipInterfaceState* device, uint32_t reserved1, uint32_t reserved2, enum MipCmd_Rtk_ServiceStatus_Serviceflags* flags, uint32_t* recievedBytes, uint32_t* lastBytes, uint64_t* lastBytesTime)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, reserved1);
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, reserved2);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_SERVICE_STATUS, buffer, cmdUsed, MIP_REPLY_DESC_RTK_SERVICE_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipCmd_Rtk_ServiceStatus_Serviceflags(buffer, sizeof(buffer), responseUsed, flags);
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, recievedBytes);
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, lastBytes);
        responseUsed = extract_u64(buffer, sizeof(buffer), responseUsed, lastBytesTime);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_ProdEraseStorage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ProdEraseStorage* self)
{
    offset = insert_MipMediaSelector(buffer, bufferSize, offset, self->media);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ProdEraseStorage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ProdEraseStorage* self)
{
    offset = extract_MipMediaSelector(buffer, bufferSize, offset, &self->media);
    
    return offset;
}


/// @brief This command will erase the selected media to a raw and unitialized state. ALL DATA WILL BE LOST.
/// This command is only available in calibration mode.
/// @param media 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_rtk_prod_erase_storage(struct MipInterfaceState* device, enum MipMediaSelector media)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipMediaSelector(buffer, sizeof(buffer), cmdUsed, media);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_PROD_ERASE_STORAGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_LedControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_LedControl* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->altColor[i]);
    offset = insert_MipLedAction(buffer, bufferSize, offset, self->act);
    offset = insert_u32(buffer, bufferSize, offset, self->period);
    
    return offset;
}

size_t extract_MipCmd_Rtk_LedControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_LedControl* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->altColor[i]);
    offset = extract_MipLedAction(buffer, bufferSize, offset, &self->act);
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
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_rtk_led_control(struct MipInterfaceState* device, const uint8_t* primaryColor, const uint8_t* altColor, enum MipLedAction act, uint32_t period)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, altColor[i]);
    cmdUsed = insert_MipLedAction(buffer, sizeof(buffer), cmdUsed, act);
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, period);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_LED_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_ModemHardReset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ModemHardReset* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_ModemHardReset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ModemHardReset* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


/// @brief This command will clear the modem flash.  THIS MUST NOT BE DONE OFTEN AS IT CAN DAMAGE THE FLASH!
/// This command is only available in calibration mode.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_rtk_modem_hard_reset(struct MipInterfaceState* device)
{
    return MipInterface_runCommand(device, MIP_RTK_COMMAND_DESC_SET, MIP_CMD_DESC_RTK_MODEM_HARD_RESET, NULL, 0);
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
