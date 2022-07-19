
#include "commands_gnss.h"

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
size_t insert_MipCmd_Gnss_ReceiverInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Gnss_ReceiverInfo_Receiverinfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_u8(buffer, bufferSize, offset, self->mip_data_descriptor_set);
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->description[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverInfo_Receiverinfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->mip_data_descriptor_set);
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->description[i]);
    
    return offset;
}


size_t insert_MipCmd_Gnss_ReceiverInfo_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_receivers);
    for(unsigned int i=0; i < self->num_receivers; i++)
        offset = insert_MipCmd_Gnss_ReceiverInfo_Receiverinfo(buffer, bufferSize, offset, &self->receiver_info[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverInfo_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo_Response* self)
{
    uint8_t num_receivers_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_receivers_local);
    if( num_receivers_local < self->num_receivers )
        self->num_receivers = num_receivers_local;
    for(unsigned int i=0; i < self->num_receivers; i++)
        offset = extract_MipCmd_Gnss_ReceiverInfo_Receiverinfo(buffer, bufferSize, offset, &self->receiver_info[i]);
    
    return offset;
}


/// @brief Return information about the GNSS receivers in the device.
/// 
/// @param[out] num_receivers Number of physical receivers in the device
/// @param[out] receiver_info 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_gnss_receiver_info(struct MipInterfaceState* device, uint8_t* num_receivers, struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* receiver_info)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_LIST_RECEIVERS, NULL, 0, MIP_REPLY_DESC_GNSS_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        uint8_t num_receivers_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_receivers_local);
        if( num_receivers_local < *num_receivers )
            *num_receivers = num_receivers_local;
        for(unsigned int i=0; i < *num_receivers; i++)
            responseUsed = extract_MipCmd_Gnss_ReceiverInfo_Receiverinfo(buffer, sizeof(buffer), responseUsed, &receiver_info[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_SignalConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_SignalConfiguration* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->gps_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->glonass_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->galileo_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_SignalConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_SignalConfiguration* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->gps_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->glonass_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->galileo_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


size_t insert_MipCmd_Gnss_SignalConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_SignalConfiguration_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->gps_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->glonass_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->galileo_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_SignalConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_SignalConfiguration_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->gps_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->glonass_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->galileo_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


/// @brief Configure the GNSS signals used by the device.
/// 
/// @param gps_enable Bitfield 0: Enable L1CA, 1: Enable L2C
/// @param glonass_enable Bitfield 0: Enable L1OF, 1: Enable L2OF
/// @param galileo_enable Bitfield 0: Enable E1,   1: Enable E5B
/// @param beidou_enable Bitfield 0: Enable B1,   1: Enable B2
/// @param reserved 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_gnss_signal_configuration(struct MipInterfaceState* device, uint8_t gps_enable, uint8_t glonass_enable, uint8_t galileo_enable, uint8_t beidou_enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, gps_enable);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, glonass_enable);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, galileo_enable);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, reserved[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// @param[out] gps_enable Bitfield 0: Enable L1CA, 1: Enable L2C
/// @param[out] glonass_enable Bitfield 0: Enable L1OF, 1: Enable L2OF
/// @param[out] galileo_enable Bitfield 0: Enable E1,   1: Enable E5B
/// @param[out] beidou_enable Bitfield 0: Enable B1,   1: Enable B2
/// @param[out] reserved 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_gnss_signal_configuration(struct MipInterfaceState* device, uint8_t* gps_enable, uint8_t* glonass_enable, uint8_t* galileo_enable, uint8_t* beidou_enable, uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, cmdUsed, MIP_REPLY_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, gps_enable);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, glonass_enable);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, galileo_enable);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, beidou_enable);
        for(unsigned int i=0; i < 4; i++)
            responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &reserved[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_gnss_signal_configuration(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_gnss_signal_configuration(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the GNSS signals used by the device.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_gnss_signal_configuration(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_SIGNAL_CONFIGURATION, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_RtkDongleConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_RtkDongleConfiguration* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_RtkDongleConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_RtkDongleConfiguration* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


size_t insert_MipCmd_Gnss_RtkDongleConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_RtkDongleConfiguration_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_RtkDongleConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_RtkDongleConfiguration_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// @param enable 0 - Disabled, 1- Enabled
/// @param reserved 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device, uint8_t enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, reserved[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// @param[out] enable 
/// @param[out] reserved 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device, uint8_t* enable, uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, cmdUsed, MIP_REPLY_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable);
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &reserved[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, cmdUsed);
}

/// @brief Configure the communications with the RTK Dongle connected to the device.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_gnss_rtk_dongle_configuration(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_RTK_DONGLE_CONFIGURATION, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_ReceiverSafeMode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverSafeMode* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverSafeMode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverSafeMode* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


/// @brief Enable/disable safe mode for the provided receiver ID.
/// Note: Receivers in safe mode will not output valid GNSS data.
/// 
/// @param receiver_id Receiver id: e.g. 1, 2, etc.
/// @param enable 0 - Disabled, 1- Enabled
/// 
/// @returns MipCmdResult
/// 
MipCmdResult gnss_receiver_safe_mode(struct MipInterfaceState* device, uint8_t receiver_id, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, receiver_id);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_GNSS_COMMAND_DESC_SET, MIP_CMD_DESC_GNSS_RECEIVER_SAFE_MODE, buffer, cmdUsed);
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus