
#include "commands_3dm.h"

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

size_t insert_MipPpsSource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipPpsSource self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipPpsSource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipPpsSource* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipSensorRangeType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipSensorRangeType self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipSensorRangeType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipSensorRangeType* self)
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
size_t insert_MipCmd_3dm_PollImuMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollImuMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollImuMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollImuMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Poll the device for an IMU message with the specified format
/// 
/// This function polls for an IMU message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set IMU Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an IMU Data packet.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the descriptor list.
/// @param descriptors Descriptor list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_3dm_poll_imu_message(struct MipInterfaceState* device, bool suppress_ack, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, suppress_ack);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_POLL_IMU_MESSAGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollGnssMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollGnssMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollGnssMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollGnssMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Poll the device for an GNSS message with the specified format
/// 
/// This function polls for a GNSS message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set GNSS Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a GNSS Data packet.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the descriptor list.
/// @param descriptors Descriptor list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_3dm_poll_gnss_message(struct MipInterfaceState* device, bool suppress_ack, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, suppress_ack);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_POLL_GNSS_MESSAGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollFilterMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollFilterMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollFilterMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollFilterMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Poll the device for an Estimation Filter message with the specified format
/// 
/// This function polls for an Estimation Filter message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Estimation Filter Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as an Estimation Filter Data packet.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the format list.
/// @param descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_3dm_poll_filter_message(struct MipInterfaceState* device, bool suppress_ack, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, suppress_ack);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_POLL_FILTER_MESSAGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollNmeaMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollNmeaMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollNmeaMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollNmeaMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


MipCmdResult mip_cmd_3dm_poll_nmea_message(struct MipInterfaceState* device, bool suppress_ack, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, suppress_ack);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_POLL_NMEA_MESSAGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_ImuMessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuMessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuMessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuMessageFormat* self)
{
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_ImuMessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuMessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuMessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuMessageFormat_Response* self)
{
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors
/// @param descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_imu_message_format(struct MipInterfaceState* device, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors
/// @param[out] descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_imu_message_format(struct MipInterfaceState* device, uint8_t* num_descriptors, struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, cmdUsed, MIP_REPLY_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        uint8_t num_descriptors_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_descriptors_local);
        if( num_descriptors_local < *num_descriptors )
            *num_descriptors = num_descriptors_local;
        for(unsigned int i=0; i < *num_descriptors; i++)
            responseUsed = extract_MipDescriptorRate(buffer, sizeof(buffer), responseUsed, &descriptors[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_imu_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_imu_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_imu_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpsMessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsMessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsMessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsMessageFormat* self)
{
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_GpsMessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsMessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsMessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsMessageFormat_Response* self)
{
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors
/// @param descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_gps_message_format(struct MipInterfaceState* device, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors
/// @param[out] descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_gps_message_format(struct MipInterfaceState* device, uint8_t* num_descriptors, struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        uint8_t num_descriptors_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_descriptors_local);
        if( num_descriptors_local < *num_descriptors )
            *num_descriptors = num_descriptors_local;
        for(unsigned int i=0; i < *num_descriptors; i++)
            responseUsed = extract_MipDescriptorRate(buffer, sizeof(buffer), responseUsed, &descriptors[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_gps_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_gps_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_gps_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_FilterMessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterMessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterMessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterMessageFormat* self)
{
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_FilterMessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterMessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterMessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterMessageFormat_Response* self)
{
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors (limited by payload size)
/// @param descriptors 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_filter_message_format(struct MipInterfaceState* device, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors (limited by payload size)
/// @param[out] descriptors 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_filter_message_format(struct MipInterfaceState* device, uint8_t* num_descriptors, struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, cmdUsed, MIP_REPLY_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        uint8_t num_descriptors_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_descriptors_local);
        if( num_descriptors_local < *num_descriptors )
            *num_descriptors = num_descriptors_local;
        for(unsigned int i=0; i < *num_descriptors; i++)
            responseUsed = extract_MipDescriptorRate(buffer, sizeof(buffer), responseUsed, &descriptors[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_filter_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_filter_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_filter_message_format(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_ImuGetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuGetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_3dm_ImuGetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuGetBaseRate_Response* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuGetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuGetBaseRate_Response* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


/// @brief Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_imu_data_base_rate(struct MipInterfaceState* device, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GET_IMU_BASE_RATE, NULL, 0, MIP_REPLY_DESC_3DM_IMU_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, rate);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpsGetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsGetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_3dm_GpsGetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsGetBaseRate_Response* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsGetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsGetBaseRate_Response* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


/// @brief Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_gnss_data_base_rate(struct MipInterfaceState* device, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GET_GNSS_BASE_RATE, NULL, 0, MIP_REPLY_DESC_3DM_GNSS_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, rate);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_FilterGetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterGetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_3dm_FilterGetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterGetBaseRate_Response* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterGetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterGetBaseRate_Response* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


/// @brief Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_estimation_filter_data_base_rate(struct MipInterfaceState* device, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    
    uint8_t responseLength = sizeof(buffer);
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GET_FILTER_BASE_RATE, NULL, 0, MIP_REPLY_DESC_3DM_FILTER_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, rate);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollData(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollData* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollData(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollData* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Poll the device for a message with the specified descriptor set and format.
/// 
/// This function polls for a message using the provided format. The resulting message
/// will maintain the order of descriptors sent in the command and any unrecognized
/// descriptors are ignored. If the format is not provided, the device will attempt to use the
/// stored format (set with the Set Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as a normal Data packet.
/// @param desc_set Data descriptor set. Must be supported.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param num_descriptors Number of descriptors in the format list.
/// @param descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_3dm_poll_data(struct MipInterfaceState* device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, suppress_ack);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_POLL_DATA, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GetBaseRate* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    
    return offset;
}

size_t extract_MipCmd_3dm_GetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GetBaseRate* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    
    return offset;
}


size_t insert_MipCmd_3dm_GetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GetBaseRate_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_GetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GetBaseRate_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


/// @brief Get the base rate for the specified descriptor set in Hz.
/// 
/// @param desc_set This is the data descriptor set. It must be a supported descriptor.
/// @param[out] desc_set Echoes the parameter in the command.
/// @param[out] rate Base rate in Hz (0 = variable, unknown, or user-defined rate.  Data will be sent when received).
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_data_base_rate(struct MipInterfaceState* device, uint8_t desc_set, uint16_t* rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GET_BASE_RATE, buffer, cmdUsed, MIP_REPLY_DESC_3DM_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &desc_set);
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, rate);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_MessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MessageFormat* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_MessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MessageFormat_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    uint8_t num_descriptors_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors_local);
    if( num_descriptors_local < self->num_descriptors )
        self->num_descriptors = num_descriptors_local;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// @param num_descriptors Number of descriptors (limited by payload size)
/// @param descriptors List of descriptors and decimations.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_message_format(struct MipInterfaceState* device, uint8_t desc_set, uint8_t num_descriptors, const struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        cmdUsed = insert_MipDescriptorRate(buffer, sizeof(buffer), cmdUsed, &descriptors[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// @param[out] desc_set Echoes the descriptor set from the command.
/// @param[out] num_descriptors Number of descriptors in the list.
/// @param[out] descriptors List of descriptors and decimations.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_message_format(struct MipInterfaceState* device, uint8_t desc_set, uint8_t* num_descriptors, struct MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, cmdUsed, MIP_REPLY_DESC_3DM_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &desc_set);
        uint8_t num_descriptors_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_descriptors_local);
        if( num_descriptors_local < *num_descriptors )
            *num_descriptors = num_descriptors_local;
        for(unsigned int i=0; i < *num_descriptors; i++)
            responseUsed = extract_MipDescriptorRate(buffer, sizeof(buffer), responseUsed, &descriptors[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_message_format(struct MipInterfaceState* device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_message_format(struct MipInterfaceState* device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, cmdUsed);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_message_format(struct MipInterfaceState* device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_DeviceSettings(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_DeviceSettings* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_DeviceSettings(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_DeviceSettings* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_device_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS, buffer, cmdUsed);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_device_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS, buffer, cmdUsed);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_device_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_DEVICE_STARTUP_SETTINGS, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_UartBaudrate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_UartBaudrate* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_3dm_UartBaudrate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_UartBaudrate* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


size_t insert_MipCmd_3dm_UartBaudrate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_UartBaudrate_Response* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_3dm_UartBaudrate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_UartBaudrate_Response* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// @param baud 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_uart_baudrate(struct MipInterfaceState* device, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u32(buffer, sizeof(buffer), cmdUsed, baud);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, cmdUsed);
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// @param[out] baud 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_uart_baudrate(struct MipInterfaceState* device, uint32_t* baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, cmdUsed, MIP_REPLY_DESC_3DM_UART_BAUDRATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u32(buffer, sizeof(buffer), responseUsed, baud);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_uart_baudrate(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, cmdUsed);
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_uart_baudrate(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, cmdUsed);
}

/// @brief Read, Save, Load, or Reset to Default the baud rate of the main communication channel.
/// 
/// For all functions except 0x01 (use new settings), the new baud rate value is ignored.
/// Please see the device user manual for supported baud rates.
/// 
/// The device will wait until all incoming and outgoing data has been sent, up
/// to a maximum of 250 ms, before applying any change.
/// 
/// No guarantee is provided as to what happens to commands issued during this
/// delay period; They may or may not be processed and any responses aren't
/// guaranteed to be at one rate or the other. The same applies to data packets.
/// 
/// It is highly recommended that the device be idle before issuing this command
/// and that it be issued in its own packet. Users should wait 250 ms after
/// sending this command before further interaction.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_uart_baudrate(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_FactoryStreaming_Action(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_FactoryStreaming_Action self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_FactoryStreaming_Action(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_FactoryStreaming_Action* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_FactoryStreaming(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FactoryStreaming* self)
{
    offset = insert_MipCmd_3dm_FactoryStreaming_Action(buffer, bufferSize, offset, self->action);
    offset = insert_u8(buffer, bufferSize, offset, self->reserved);
    
    return offset;
}

size_t extract_MipCmd_3dm_FactoryStreaming(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FactoryStreaming* self)
{
    offset = extract_MipCmd_3dm_FactoryStreaming_Action(buffer, bufferSize, offset, &self->action);
    offset = extract_u8(buffer, bufferSize, offset, &self->reserved);
    
    return offset;
}


/// @brief Configures the device for recording data for technical support.
/// 
/// This command will configure all available data streams to predefined
/// formats designed to be used with technical support.
/// @param action 
/// @param reserved Reserved. Set to 0x00.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult mip_cmd_3dm_factory_streaming(struct MipInterfaceState* device, enum MipCmd_3dm_FactoryStreaming_Action action, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipCmd_3dm_FactoryStreaming_Action(buffer, sizeof(buffer), cmdUsed, action);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, reserved);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CONFIGURE_FACTORY_STREAMING, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_DatastreamControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_DatastreamControl* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_3dm_DatastreamControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_DatastreamControl* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_MipCmd_3dm_DatastreamControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_DatastreamControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_bool(buffer, bufferSize, offset, self->enabled);
    
    return offset;
}

size_t extract_MipCmd_3dm_DatastreamControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_DatastreamControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_bool(buffer, bufferSize, offset, &self->enabled);
    
    return offset;
}


/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// @param enable True or false to enable or disable the stream.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_datastream_control(struct MipInterfaceState* device, uint8_t desc_set, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, enable);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, cmdUsed);
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// @param[out] desc_set 
/// @param[out] enabled 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_datastream_control(struct MipInterfaceState* device, uint8_t desc_set, bool* enabled)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, cmdUsed, MIP_REPLY_DESC_3DM_DATASTREAM_ENABLE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &desc_set);
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, enabled);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_datastream_control(struct MipInterfaceState* device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, cmdUsed);
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_datastream_control(struct MipInterfaceState* device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, cmdUsed);
}

/// @brief Enable/disable the selected data stream.
/// 
/// Each data stream (descriptor set) can be enabled or disabled.
/// The default for the device is all streams enabled.
/// For all functions except 0x01 (use new setting),
/// the new enable flag value is ignored and can be omitted.
/// @param desc_set The descriptor set of the stream to control. When function is SAVE, LOAD, or DEFAULT, can be ALL_STREAMS(0) to apply to all descriptor sets. On Generation 5 products, this must be one of the above legacy constants.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_datastream_control(struct MipInterfaceState* device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, desc_set);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GnssSbasSettings_Sbasoptions self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GnssSbasSettings_Sbasoptions* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_3dm_GnssSbasSettings(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssSbasSettings* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable_sbas);
    offset = insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, self->sbas_options);
    offset = insert_u8(buffer, bufferSize, offset, self->num_included_prns);
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->included_prns[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssSbasSettings(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssSbasSettings* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable_sbas);
    offset = extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, &self->sbas_options);
    uint8_t num_included_prns_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_included_prns_local);
    if( num_included_prns_local < self->num_included_prns )
        self->num_included_prns = num_included_prns_local;
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->included_prns[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_GnssSbasSettings_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssSbasSettings_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable_sbas);
    offset = insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, self->sbas_options);
    offset = insert_u8(buffer, bufferSize, offset, self->num_included_prns);
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->included_prns[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssSbasSettings_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssSbasSettings_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable_sbas);
    offset = extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, &self->sbas_options);
    uint8_t num_included_prns_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_included_prns_local);
    if( num_included_prns_local < self->num_included_prns )
        self->num_included_prns = num_included_prns_local;
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->included_prns[i]);
    
    return offset;
}


/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// @param enable_sbas 0 - SBAS Disabled, 1 - SBAS enabled
/// @param sbas_options SBAS options, see definition
/// @param num_included_prns Number of SBAS PRNs to include in search (0 = include all)
/// @param included_prns List of specific SBAS PRNs to search for
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_sbas_settings(struct MipInterfaceState* device, uint8_t enable_sbas, enum MipCmd_3dm_GnssSbasSettings_Sbasoptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, enable_sbas);
    cmdUsed = insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, sizeof(buffer), cmdUsed, sbas_options);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, num_included_prns);
    for(unsigned int i=0; i < num_included_prns; i++)
        cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, included_prns[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, cmdUsed);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// @param[out] enable_sbas 0 - SBAS Disabled, 1 - SBAS enabled
/// @param[out] sbas_options SBAS options, see definition
/// @param[out] num_included_prns Number of SBAS PRNs to include in search (0 = include all)
/// @param[out] included_prns List of specific SBAS PRNs to search for
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_sbas_settings(struct MipInterfaceState* device, uint8_t* enable_sbas, enum MipCmd_3dm_GnssSbasSettings_Sbasoptions* sbas_options, uint8_t* num_included_prns, uint16_t* included_prns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, enable_sbas);
        responseUsed = extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, sizeof(buffer), responseUsed, sbas_options);
        uint8_t num_included_prns_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_included_prns_local);
        if( num_included_prns_local < *num_included_prns )
            *num_included_prns = num_included_prns_local;
        for(unsigned int i=0; i < *num_included_prns; i++)
            responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, &included_prns[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_sbas_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, cmdUsed);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_sbas_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, cmdUsed);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_sbas_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_SBAS_SETTINGS, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GnssTimeAssistance(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssTimeAssistance* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_float(buffer, bufferSize, offset, self->accuracy);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssTimeAssistance(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssTimeAssistance* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_float(buffer, bufferSize, offset, &self->accuracy);
    
    return offset;
}


size_t insert_MipCmd_3dm_GnssTimeAssistance_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssTimeAssistance_Response* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_float(buffer, bufferSize, offset, self->accuracy);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssTimeAssistance_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssTimeAssistance_Response* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_float(buffer, bufferSize, offset, &self->accuracy);
    
    return offset;
}


/// @brief Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
/// @param tow GPS Time of week [seconds]
/// @param week_number GPS Weeks since 1980 [weeks]
/// @param accuracy Accuracy of time information [seconds]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_gnss_time_assistance(struct MipInterfaceState* device, double tow, uint16_t week_number, float accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_double(buffer, sizeof(buffer), cmdUsed, tow);
    cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, week_number);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, accuracy);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE, buffer, cmdUsed);
}

/// @brief Provide the GNSS subsystem with initial time information.
/// 
/// This message is required immediately after power up if GNSS Assist was enabled when the device was powered off.
/// This will initialize the subsystem clock to help reduce the time to first fix (TTFF).
/// @param[out] tow GPS Time of week [seconds]
/// @param[out] week_number GPS Weeks since 1980 [weeks]
/// @param[out] accuracy Accuracy of time information [seconds]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_gnss_time_assistance(struct MipInterfaceState* device, double* tow, uint16_t* week_number, float* accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GNSS_TIME_ASSISTANCE, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GNSS_TIME_ASSISTANCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_double(buffer, sizeof(buffer), responseUsed, tow);
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, week_number);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, accuracy);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_AdvLowpassFilter(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AdvLowpassFilter* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->target_descriptor);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    offset = insert_bool(buffer, bufferSize, offset, self->manual);
    offset = insert_u16(buffer, bufferSize, offset, self->frequency);
    offset = insert_u8(buffer, bufferSize, offset, self->reserved);
    
    return offset;
}

size_t extract_MipCmd_3dm_AdvLowpassFilter(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AdvLowpassFilter* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->target_descriptor);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->manual);
    offset = extract_u16(buffer, bufferSize, offset, &self->frequency);
    offset = extract_u8(buffer, bufferSize, offset, &self->reserved);
    
    return offset;
}


size_t insert_MipCmd_3dm_AdvLowpassFilter_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AdvLowpassFilter_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->target_descriptor);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    offset = insert_bool(buffer, bufferSize, offset, self->manual);
    offset = insert_u16(buffer, bufferSize, offset, self->frequency);
    offset = insert_u8(buffer, bufferSize, offset, self->reserved);
    
    return offset;
}

size_t extract_MipCmd_3dm_AdvLowpassFilter_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AdvLowpassFilter_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->target_descriptor);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->manual);
    offset = extract_u16(buffer, bufferSize, offset, &self->frequency);
    offset = extract_u8(buffer, bufferSize, offset, &self->reserved);
    
    return offset;
}


/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// @param enable The target data will be filtered if this is true.
/// @param manual If false, the cutoff frequency is set to half of the streaming rate as configured by the message format command. Otherwise, the cutoff frequency is set according to the following 'frequency' parameter.
/// @param frequency -3dB cutoff frequency in Hz. Will not affect filtering if 'manual' is false.
/// @param reserved Reserved, set to 0x00.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_advanced_low_pass_filter_settings(struct MipInterfaceState* device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, target_descriptor);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, enable);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, manual);
    cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, frequency);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, reserved);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, cmdUsed);
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// @param[out] target_descriptor 
/// @param[out] enable True if the filter is currently enabled.
/// @param[out] manual True if the filter cutoff was manually configured.
/// @param[out] frequency The cutoff frequency of the filter. If the filter is in auto mode, this value is unspecified.
/// @param[out] reserved Reserved and must be ignored.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_advanced_low_pass_filter_settings(struct MipInterfaceState* device, uint8_t target_descriptor, bool* enable, bool* manual, uint16_t* frequency, uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, target_descriptor);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, cmdUsed, MIP_REPLY_DESC_3DM_ADVANCED_DATA_FILTER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &target_descriptor);
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, enable);
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, manual);
        responseUsed = extract_u16(buffer, sizeof(buffer), responseUsed, frequency);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, reserved);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_advanced_low_pass_filter_settings(struct MipInterfaceState* device, uint8_t target_descriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, target_descriptor);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, cmdUsed);
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_advanced_low_pass_filter_settings(struct MipInterfaceState* device, uint8_t target_descriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, target_descriptor);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, cmdUsed);
}

/// @brief Advanced configuration for the IMU data quantity low-pass filters.
/// 
/// The scaled data quantities are by default filtered through a single-pole IIR low-pass filter
/// which is configured with a -3dB cutoff frequency of half the reporting frequency (set by
/// decimation factor in the IMU Message Format command) to prevent aliasing on a per data
/// quantity basis. This advanced configuration command allows for the cutoff frequency to
/// be configured independently of the data reporting frequency as well as allowing for a
/// complete bypass of the digital low-pass filter.
/// 
/// Possible data descriptors:
/// 0x04 – Scaled accelerometer data
/// 0x05 – Scaled gyro data
/// 0x06 – Scaled magnetometer data (if applicable)
/// 0x17 – Scaled pressure data (if applicable)
/// @param target_descriptor Field descriptor of filtered quantity within the Sensor data set. Supported values are accel (0x04), gyro (0x05), mag (0x06), and pressure (0x17), provided the data is supported by the device. Except with the READ function selector, this can be 0 to apply to all of the above quantities.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_advanced_low_pass_filter_settings(struct MipInterfaceState* device, uint8_t target_descriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, target_descriptor);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ADVANCED_DATA_FILTER, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PpsSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PpsSource* self)
{
    offset = insert_MipPpsSource(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_3dm_PpsSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PpsSource* self)
{
    offset = extract_MipPpsSource(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


size_t insert_MipCmd_3dm_PpsSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PpsSource_Response* self)
{
    offset = insert_MipPpsSource(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_3dm_PpsSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PpsSource_Response* self)
{
    offset = extract_MipPpsSource(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// @param source 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_mip_cmd_3dm_pps_source(struct MipInterfaceState* device, enum MipPpsSource source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_MipPpsSource(buffer, sizeof(buffer), cmdUsed, source);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, cmdUsed);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// @param[out] source 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_mip_cmd_3dm_pps_source(struct MipInterfaceState* device, enum MipPpsSource* source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, cmdUsed, MIP_REPLY_DESC_3DM_PPS_SOURCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipPpsSource(buffer, sizeof(buffer), responseUsed, source);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_mip_cmd_3dm_pps_source(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, cmdUsed);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_mip_cmd_3dm_pps_source(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, cmdUsed);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_mip_cmd_3dm_pps_source(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_PPS_SOURCE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpioConfig_Feature(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GpioConfig_Feature self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GpioConfig_Feature(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GpioConfig_Feature* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_GpioConfig_Behavior(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GpioConfig_Behavior self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GpioConfig_Behavior(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GpioConfig_Behavior* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_GpioConfig_Pinmode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GpioConfig_Pinmode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GpioConfig_Pinmode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GpioConfig_Pinmode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_3dm_GpioConfig(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioConfig* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, self->feature);
    offset = insert_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, self->behavior);
    offset = insert_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, self->pin_mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioConfig(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioConfig* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, &self->feature);
    offset = extract_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, &self->behavior);
    offset = extract_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, &self->pin_mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_GpioConfig_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioConfig_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, self->feature);
    offset = insert_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, self->behavior);
    offset = insert_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, self->pin_mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioConfig_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioConfig_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, &self->feature);
    offset = extract_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, &self->behavior);
    offset = extract_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, &self->pin_mode);
    
    return offset;
}


/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// @param feature Determines how the pin will be used.
/// @param behavior Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
/// @param pin_mode GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_gpio_configuration(struct MipInterfaceState* device, uint8_t pin, enum MipCmd_3dm_GpioConfig_Feature feature, enum MipCmd_3dm_GpioConfig_Behavior behavior, enum MipCmd_3dm_GpioConfig_Pinmode pin_mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    cmdUsed = insert_MipCmd_3dm_GpioConfig_Feature(buffer, sizeof(buffer), cmdUsed, feature);
    cmdUsed = insert_MipCmd_3dm_GpioConfig_Behavior(buffer, sizeof(buffer), cmdUsed, behavior);
    cmdUsed = insert_MipCmd_3dm_GpioConfig_Pinmode(buffer, sizeof(buffer), cmdUsed, pin_mode);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, cmdUsed);
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// @param[out] pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// @param[out] feature Determines how the pin will be used.
/// @param[out] behavior Select an appropriate value from the enumeration based on the selected feature (e.g. for PPS, select one of the values prefixed with PPS_.)
/// @param[out] pin_mode GPIO configuration. May be restricted depending on device, pin, feature, and behavior. See device user manual.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_gpio_configuration(struct MipInterfaceState* device, uint8_t pin, enum MipCmd_3dm_GpioConfig_Feature* feature, enum MipCmd_3dm_GpioConfig_Behavior* behavior, enum MipCmd_3dm_GpioConfig_Pinmode* pin_mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GPIO_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &pin);
        responseUsed = extract_MipCmd_3dm_GpioConfig_Feature(buffer, sizeof(buffer), responseUsed, feature);
        responseUsed = extract_MipCmd_3dm_GpioConfig_Behavior(buffer, sizeof(buffer), responseUsed, behavior);
        responseUsed = extract_MipCmd_3dm_GpioConfig_Pinmode(buffer, sizeof(buffer), responseUsed, pin_mode);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_gpio_configuration(struct MipInterfaceState* device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, cmdUsed);
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_gpio_configuration(struct MipInterfaceState* device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, cmdUsed);
}

/// @brief Configures the user GPIO pins on the connector for use with several built-in functions or for general input or output.
/// 
/// GPIO pins are device-dependent. Some features are only available on
/// certain pins. Some behaviors require specific configurations.
/// Consult the device user manual for restrictions and default settings.
/// 
/// To avoid glitches on GPIOs configured as an output in a mode other than
/// GPIO, always configure the relevant function before setting up the pin
/// with this command. Otherwise, the pin state will be undefined between
/// this command and the one to set up the feature. For input pins, use
/// this command first so the state is well-defined when the feature is
/// initialized.
/// 
/// Some configurations can only be active on one pin at a time. If such
/// configuration is applied to a second pin, the second one will take
/// precedence and the original pin's configuration will be reset.
/// 
/// @param pin GPIO pin number counting from 1. For save, load, and default function selectors, this can be 0 to select all pins.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_gpio_configuration(struct MipInterfaceState* device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_CONFIG, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpioState(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioState* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_bool(buffer, bufferSize, offset, self->state);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioState(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioState* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_bool(buffer, bufferSize, offset, &self->state);
    
    return offset;
}


size_t insert_MipCmd_3dm_GpioState_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioState_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_bool(buffer, bufferSize, offset, self->state);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioState_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioState_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_bool(buffer, bufferSize, offset, &self->state);
    
    return offset;
}


/// @brief Allows the state of the pin to be read or controlled.
/// 
/// This command serves two purposes: 1) To allow reading the state of a pin via command,
/// rather than polling a data quantity, and 2) to provide a way to set the output state
/// without also having to specify the operating mode.
/// 
/// The state read back from the pin is the physical state of the pin, rather than a
/// configuration value. The state can be read regardless of its configuration as long as
/// the device supports GPIO input on that pin. If the pin is set to an output, the read
/// value would match the output value.
/// 
/// While the state of a pin can always be set, it will only have an observable effect if
/// the pin is set to output mode.
/// 
/// This command does not support saving, loading, or reseting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
/// @param pin GPIO pin number counting from 1. Cannot be 0.
/// @param state The pin state.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_gpio_state(struct MipInterfaceState* device, uint8_t pin, bool state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, state);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_STATE, buffer, cmdUsed);
}

/// @brief Allows the state of the pin to be read or controlled.
/// 
/// This command serves two purposes: 1) To allow reading the state of a pin via command,
/// rather than polling a data quantity, and 2) to provide a way to set the output state
/// without also having to specify the operating mode.
/// 
/// The state read back from the pin is the physical state of the pin, rather than a
/// configuration value. The state can be read regardless of its configuration as long as
/// the device supports GPIO input on that pin. If the pin is set to an output, the read
/// value would match the output value.
/// 
/// While the state of a pin can always be set, it will only have an observable effect if
/// the pin is set to output mode.
/// 
/// This command does not support saving, loading, or reseting the state. Instead, use the
/// GPIO Configuration command, which allows the initial state to be configured.
/// @param pin GPIO pin number counting from 1. Cannot be 0.
/// @param[out] pin GPIO pin number counting from 1. Cannot be 0.
/// @param[out] state The pin state.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_gpio_state(struct MipInterfaceState* device, uint8_t pin, bool* state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, pin);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GPIO_STATE, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GPIO_STATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &pin);
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, state);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Odometer_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_Odometer_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_Odometer_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_Odometer_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_Odometer(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Odometer* self)
{
    offset = insert_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, self->mode);
    offset = insert_float(buffer, bufferSize, offset, self->scaling);
    offset = insert_float(buffer, bufferSize, offset, self->uncertainty);
    
    return offset;
}

size_t extract_MipCmd_3dm_Odometer(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Odometer* self)
{
    offset = extract_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, &self->mode);
    offset = extract_float(buffer, bufferSize, offset, &self->scaling);
    offset = extract_float(buffer, bufferSize, offset, &self->uncertainty);
    
    return offset;
}


size_t insert_MipCmd_3dm_Odometer_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Odometer_Response* self)
{
    offset = insert_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, self->mode);
    offset = insert_float(buffer, bufferSize, offset, self->scaling);
    offset = insert_float(buffer, bufferSize, offset, self->uncertainty);
    
    return offset;
}

size_t extract_MipCmd_3dm_Odometer_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Odometer_Response* self)
{
    offset = extract_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, &self->mode);
    offset = extract_float(buffer, bufferSize, offset, &self->scaling);
    offset = extract_float(buffer, bufferSize, offset, &self->uncertainty);
    
    return offset;
}


/// @brief Configures the hardware odometer interface.
/// 
/// @param mode Mode setting.
/// @param scaling Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
/// @param uncertainty Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_odometer_settings(struct MipInterfaceState* device, enum MipCmd_3dm_Odometer_Mode mode, float scaling, float uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_MipCmd_3dm_Odometer_Mode(buffer, sizeof(buffer), cmdUsed, mode);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, scaling);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, uncertainty);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, cmdUsed);
}

/// @brief Configures the hardware odometer interface.
/// 
/// @param[out] mode Mode setting.
/// @param[out] scaling Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
/// @param[out] uncertainty Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_odometer_settings(struct MipInterfaceState* device, enum MipCmd_3dm_Odometer_Mode* mode, float* scaling, float* uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, cmdUsed, MIP_REPLY_DESC_3DM_ODOMETER_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipCmd_3dm_Odometer_Mode(buffer, sizeof(buffer), responseUsed, mode);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, scaling);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, uncertainty);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_odometer_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, cmdUsed);
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_odometer_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, cmdUsed);
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_odometer_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ODOMETER_CONFIG, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventSupport_Query(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventSupport_Query self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventSupport_Query(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventSupport_Query* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventSupport(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventSupport* self)
{
    offset = insert_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, self->query);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventSupport(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventSupport* self)
{
    offset = extract_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, &self->query);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventSupport_Info(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventSupport_Info* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventSupport_Info(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventSupport_Info* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventSupport_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventSupport_Response* self)
{
    offset = insert_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, self->query);
    offset = insert_u8(buffer, bufferSize, offset, self->max_instances);
    offset = insert_u8(buffer, bufferSize, offset, self->num_entries);
    assert(self->num_entries <= 0);
    for(unsigned int i=0; i < self->num_entries; i++)
        offset = insert_MipCmd_3dm_EventSupport_Info(buffer, bufferSize, offset, &self->entries[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventSupport_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventSupport_Response* self)
{
    offset = extract_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, &self->query);
    offset = extract_u8(buffer, bufferSize, offset, &self->max_instances);
    uint8_t num_entries_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_entries_local);
    if( num_entries_local < self->num_entries )
        self->num_entries = num_entries_local;
    for(unsigned int i=0; i < self->num_entries; i++)
        offset = extract_MipCmd_3dm_EventSupport_Info(buffer, bufferSize, offset, &self->entries[i]);
    
    return offset;
}


/// @brief Lists the available trigger or action types.
/// 
/// There are a limited number of trigger and action slots available
/// in the device. Up to M triggers and N actions can be configured at once
/// in slots 1..M and 1..N respectively. M and N are identified by the
/// max_instances field in the response with the appropriate query selector.
/// 
/// Each slot can be configured as one of a variety of different types of
/// triggers or actions. The supported types are enumerated in the response
/// to this command. Additionally, there is a limit on the number of a given
/// type. In other words, while the device may support M triggers in total,
/// only a few of them maybe usable as a given type. This limit helps optimize
/// device resources. The limit is identified in the count field.
/// 
/// All of the information in this command is available in the user manual.
/// This command provides a programmatic method for obtaining the information.
/// 
/// @param query What type of information to retrieve.
/// @param[out] query Query type specified in the command.
/// @param[out] max_instances Number of slots available. The 'instance' number for the configuration or control commands must be between 1 and this value.
/// @param[out] num_entries Number of supported types.
/// @param[out] entries List of supported types.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_supported_events(struct MipInterfaceState* device, enum MipCmd_3dm_EventSupport_Query query, uint8_t* max_instances, uint8_t* num_entries, struct MipCmd_3dm_EventSupport_Info* entries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipCmd_3dm_EventSupport_Query(buffer, sizeof(buffer), cmdUsed, query);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_SUPPORT, buffer, cmdUsed, MIP_REPLY_DESC_3DM_EVENT_SUPPORT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipCmd_3dm_EventSupport_Query(buffer, sizeof(buffer), responseUsed, &query);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, max_instances);
        uint8_t num_entries_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_entries_local);
        if( num_entries_local < *num_entries )
            *num_entries = num_entries_local;
        for(unsigned int i=0; i < *num_entries; i++)
            responseUsed = extract_MipCmd_3dm_EventSupport_Info(buffer, sizeof(buffer), responseUsed, &entries[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventControl_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventControl_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventControl_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventControl_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventControl* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventControl* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// @param mode How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_event_control(struct MipInterfaceState* device, uint8_t instance, enum MipCmd_3dm_EventControl_Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    cmdUsed = insert_MipCmd_3dm_EventControl_Mode(buffer, sizeof(buffer), cmdUsed, mode);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, cmdUsed);
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// @param[out] instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// @param[out] mode How to change the trigger state. Except when instance is 0, the corresponding trigger must be configured, i.e. not have type 0.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_event_control(struct MipInterfaceState* device, uint8_t instance, enum MipCmd_3dm_EventControl_Mode* mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, cmdUsed, MIP_REPLY_DESC_3DM_EVENT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &instance);
        responseUsed = extract_MipCmd_3dm_EventControl_Mode(buffer, sizeof(buffer), responseUsed, mode);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_event_control(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, cmdUsed);
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_event_control(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, cmdUsed);
}

/// @brief Enables or disables event triggers.
/// 
/// Triggers can be disabled, enabled, and tested. While disabled, a trigger will
/// not evaluate its logic and effective behave like no trigger is configured.
/// A disabled trigger will not activate any actions. Triggers are disabled by default.
/// 
/// Use this command to enable (or disable) a trigger, or to place it into a test mode.
/// When in test mode, the trigger logic is disabled but the output is forced to
/// the active state, meaning that it will behave as if the trigger logic is satisfied
/// and any associated actions will execute.
/// @param instance Trigger instance to affect. 0 can be used to apply the mode to all configured triggers, except when the function selector is READ.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_event_control(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_CONTROL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventTriggerStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTriggerStatus* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->requested_instances[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTriggerStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTriggerStatus* self)
{
    uint8_t requested_count_local;
    offset = extract_u8(buffer, bufferSize, offset, &requested_count_local);
    if( requested_count_local < self->requested_count )
        self->requested_count = requested_count_local;
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->requested_instances[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTriggerStatus_Status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTriggerStatus_Status self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTriggerStatus_Status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTriggerStatus_Status* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_3dm_EventTriggerStatus_Entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTriggerStatus_Entry* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    offset = insert_MipCmd_3dm_EventTriggerStatus_Status(buffer, bufferSize, offset, self->status);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTriggerStatus_Entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTriggerStatus_Entry* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    offset = extract_MipCmd_3dm_EventTriggerStatus_Status(buffer, bufferSize, offset, &self->status);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTriggerStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTriggerStatus_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    for(unsigned int i=0; i < self->count; i++)
        offset = insert_MipCmd_3dm_EventTriggerStatus_Entry(buffer, bufferSize, offset, &self->triggers[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTriggerStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTriggerStatus_Response* self)
{
    uint8_t count_local;
    offset = extract_u8(buffer, bufferSize, offset, &count_local);
    if( count_local < self->count )
        self->count = count_local;
    for(unsigned int i=0; i < self->count; i++)
        offset = extract_MipCmd_3dm_EventTriggerStatus_Entry(buffer, bufferSize, offset, &self->triggers[i]);
    
    return offset;
}


MipCmdResult get_trigger_status(struct MipInterfaceState* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count, struct MipCmd_3dm_EventTriggerStatus_Entry* triggers)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, requested_count);
    for(unsigned int i=0; i < requested_count; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, requested_instances[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_STATUS, buffer, cmdUsed, MIP_REPLY_DESC_3DM_EVENT_TRIGGER_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        uint8_t count_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &count_local);
        if( count_local < *count )
            *count = count_local;
        for(unsigned int i=0; i < *count; i++)
            responseUsed = extract_MipCmd_3dm_EventTriggerStatus_Entry(buffer, sizeof(buffer), responseUsed, &triggers[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventActionStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventActionStatus* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->requested_instances[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventActionStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventActionStatus* self)
{
    uint8_t requested_count_local;
    offset = extract_u8(buffer, bufferSize, offset, &requested_count_local);
    if( requested_count_local < self->requested_count )
        self->requested_count = requested_count_local;
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->requested_instances[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventActionStatus_Entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventActionStatus_Entry* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->action_type);
    offset = insert_u8(buffer, bufferSize, offset, self->trigger_id);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventActionStatus_Entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventActionStatus_Entry* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->action_type);
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger_id);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventActionStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventActionStatus_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    for(unsigned int i=0; i < self->count; i++)
        offset = insert_MipCmd_3dm_EventActionStatus_Entry(buffer, bufferSize, offset, &self->actions[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventActionStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventActionStatus_Response* self)
{
    uint8_t count_local;
    offset = extract_u8(buffer, bufferSize, offset, &count_local);
    if( count_local < self->count )
        self->count = count_local;
    for(unsigned int i=0; i < self->count; i++)
        offset = extract_MipCmd_3dm_EventActionStatus_Entry(buffer, bufferSize, offset, &self->actions[i]);
    
    return offset;
}


MipCmdResult get_action_status(struct MipInterfaceState* device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t* count, struct MipCmd_3dm_EventActionStatus_Entry* actions)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, requested_count);
    for(unsigned int i=0; i < requested_count; i++)
        cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, requested_instances[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_STATUS, buffer, cmdUsed, MIP_REPLY_DESC_3DM_EVENT_ACTION_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        uint8_t count_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &count_local);
        if( count_local < *count )
            *count = count_local;
        for(unsigned int i=0; i < *count; i++)
            responseUsed = extract_MipCmd_3dm_EventActionStatus_Entry(buffer, sizeof(buffer), responseUsed, &actions[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventTrigger_Type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTrigger_Type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTrigger_Type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTrigger_Type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventTrigger_Gpioparams_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTrigger_Gpioparams_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTrigger_Gpioparams_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTrigger_Gpioparams_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventTrigger_Gpioparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Gpioparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_EventTrigger_Gpioparams_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Gpioparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Gpioparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_EventTrigger_Gpioparams_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger_Thresholdparams_Type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTrigger_Thresholdparams_Type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTrigger_Thresholdparams_Type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTrigger_Thresholdparams_Type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventTrigger_Thresholdparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Thresholdparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u8(buffer, bufferSize, offset, self->field_desc);
    offset = insert_u8(buffer, bufferSize, offset, self->param_id);
    offset = insert_MipCmd_3dm_EventTrigger_Thresholdparams_Type(buffer, bufferSize, offset, self->type);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_WINDOW )
        offset = insert_double(buffer, bufferSize, offset, self->low_thres);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_INTERVAL )
        offset = insert_double(buffer, bufferSize, offset, self->int_thres);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_WINDOW )
        offset = insert_double(buffer, bufferSize, offset, self->high_thres);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_INTERVAL )
        offset = insert_double(buffer, bufferSize, offset, self->interval);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Thresholdparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Thresholdparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_u8(buffer, bufferSize, offset, &self->field_desc);
    offset = extract_u8(buffer, bufferSize, offset, &self->param_id);
    offset = extract_MipCmd_3dm_EventTrigger_Thresholdparams_Type(buffer, bufferSize, offset, &self->type);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_WINDOW )
        offset = extract_double(buffer, bufferSize, offset, &self->low_thres);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_INTERVAL )
        offset = extract_double(buffer, bufferSize, offset, &self->int_thres);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_WINDOW )
        offset = extract_double(buffer, bufferSize, offset, &self->high_thres);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_THRESHOLDPARAMS_TYPE_INTERVAL )
        offset = extract_double(buffer, bufferSize, offset, &self->interval);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger_Combinationparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Combinationparams* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->logic_table);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->input_triggers[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Combinationparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Combinationparams* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->logic_table);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->input_triggers[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, self->type);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_GPIO )
        offset = insert_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_THRESHOLD )
        offset = insert_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_COMBINATION )
        offset = insert_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, &self->type);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_GPIO )
        offset = extract_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_THRESHOLD )
        offset = extract_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_COMBINATION )
        offset = extract_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, self->type);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_GPIO )
        offset = insert_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_THRESHOLD )
        offset = insert_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_COMBINATION )
        offset = insert_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, &self->type);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_GPIO )
        offset = extract_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_THRESHOLD )
        offset = extract_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    if( self->type == MIPCMD_3DM_EVENTTRIGGER_TYPE_COMBINATION )
        offset = extract_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}


/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param type Type of trigger to configure.
/// @param  
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_event_trigger_configuration(struct MipInterfaceState* device, uint8_t instance, enum MipCmd_3dm_EventTrigger_Type type, const void* gpio_threshold_combination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    cmdUsed = insert_MipCmd_3dm_EventTrigger_Type(buffer, sizeof(buffer), cmdUsed, type);
    if( type == MIPCMD_3DM_EVENTTRIGGER_TYPE_GPIO )
        cmdUsed = insert_MipCmd_3dm_EventTrigger_Gpioparams(buffer, sizeof(buffer), cmdUsed, gpio_threshold_combination);
    if( type == MIPCMD_3DM_EVENTTRIGGER_TYPE_THRESHOLD )
        cmdUsed = insert_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, sizeof(buffer), cmdUsed, gpio_threshold_combination);
    if( type == MIPCMD_3DM_EVENTTRIGGER_TYPE_COMBINATION )
        cmdUsed = insert_MipCmd_3dm_EventTrigger_Combinationparams(buffer, sizeof(buffer), cmdUsed, gpio_threshold_combination);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, cmdUsed);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] type Type of trigger to configure.
/// @param[out]  
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_event_trigger_configuration(struct MipInterfaceState* device, uint8_t instance, enum MipCmd_3dm_EventTrigger_Type* type, void* gpio_threshold_combination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, cmdUsed, MIP_REPLY_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &instance);
        responseUsed = extract_MipCmd_3dm_EventTrigger_Type(buffer, sizeof(buffer), responseUsed, type);
        if( *type == MIPCMD_3DM_EVENTTRIGGER_TYPE_GPIO )
            responseUsed = extract_MipCmd_3dm_EventTrigger_Gpioparams(buffer, sizeof(buffer), responseUsed, gpio_threshold_combination);
        if( *type == MIPCMD_3DM_EVENTTRIGGER_TYPE_THRESHOLD )
            responseUsed = extract_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, sizeof(buffer), responseUsed, gpio_threshold_combination);
        if( *type == MIPCMD_3DM_EVENTTRIGGER_TYPE_COMBINATION )
            responseUsed = extract_MipCmd_3dm_EventTrigger_Combinationparams(buffer, sizeof(buffer), responseUsed, gpio_threshold_combination);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_event_trigger_configuration(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, cmdUsed);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_event_trigger_configuration(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, cmdUsed);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_event_trigger_configuration(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_TRIGGER_CONFIG, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventAction_Type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventAction_Type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventAction_Type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventAction_Type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventAction_Gpioparams_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventAction_Gpioparams_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventAction_Gpioparams_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventAction_Gpioparams_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventAction_Gpioparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction_Gpioparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_EventAction_Gpioparams_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction_Gpioparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction_Gpioparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_EventAction_Gpioparams_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventAction_Messageparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction_Messageparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u16(buffer, bufferSize, offset, self->decimation);
    offset = insert_u8(buffer, bufferSize, offset, self->num_fields);
    for(unsigned int i=0; i < self->num_fields; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction_Messageparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction_Messageparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_u16(buffer, bufferSize, offset, &self->decimation);
    uint8_t num_fields_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_fields_local);
    if( num_fields_local < self->num_fields )
        self->num_fields = num_fields_local;
    for(unsigned int i=0; i < self->num_fields; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventAction(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_u8(buffer, bufferSize, offset, self->trigger);
    offset = insert_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, self->type);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_GPIO )
        offset = insert_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_MESSAGE )
        offset = insert_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger);
    offset = extract_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, &self->type);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_GPIO )
        offset = extract_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_MESSAGE )
        offset = extract_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventAction_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_u8(buffer, bufferSize, offset, self->trigger);
    offset = insert_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, self->type);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_GPIO )
        offset = insert_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_MESSAGE )
        offset = insert_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger);
    offset = extract_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, &self->type);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_GPIO )
        offset = extract_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    if( self->type == MIPCMD_3DM_EVENTACTION_TYPE_MESSAGE )
        offset = extract_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}


/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param trigger Trigger ID number.
/// @param type Type of action to configure.
/// @param  
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_event_action_configuration(struct MipInterfaceState* device, uint8_t instance, uint8_t trigger, enum MipCmd_3dm_EventAction_Type type, const void* gpio_message)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, trigger);
    cmdUsed = insert_MipCmd_3dm_EventAction_Type(buffer, sizeof(buffer), cmdUsed, type);
    if( type == MIPCMD_3DM_EVENTACTION_TYPE_GPIO )
        cmdUsed = insert_MipCmd_3dm_EventAction_Gpioparams(buffer, sizeof(buffer), cmdUsed, gpio_message);
    if( type == MIPCMD_3DM_EVENTACTION_TYPE_MESSAGE )
        cmdUsed = insert_MipCmd_3dm_EventAction_Messageparams(buffer, sizeof(buffer), cmdUsed, gpio_message);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, cmdUsed);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] trigger Trigger ID number.
/// @param[out] type Type of action to configure.
/// @param[out]  
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_event_action_configuration(struct MipInterfaceState* device, uint8_t instance, uint8_t* trigger, enum MipCmd_3dm_EventAction_Type* type, void* gpio_message)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, cmdUsed, MIP_REPLY_DESC_3DM_EVENT_ACTION_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &instance);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, trigger);
        responseUsed = extract_MipCmd_3dm_EventAction_Type(buffer, sizeof(buffer), responseUsed, type);
        if( *type == MIPCMD_3DM_EVENTACTION_TYPE_GPIO )
            responseUsed = extract_MipCmd_3dm_EventAction_Gpioparams(buffer, sizeof(buffer), responseUsed, gpio_message);
        if( *type == MIPCMD_3DM_EVENTACTION_TYPE_MESSAGE )
            responseUsed = extract_MipCmd_3dm_EventAction_Messageparams(buffer, sizeof(buffer), responseUsed, gpio_message);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_event_action_configuration(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, cmdUsed);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_event_action_configuration(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, cmdUsed);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_event_action_configuration(struct MipInterfaceState* device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, instance);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_EVENT_ACTION_CONFIG, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_AccelBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AccelBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_AccelBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AccelBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_AccelBias_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AccelBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_AccelBias_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AccelBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// @param bias accelerometer bias in the sensor frame (x,y,z) [g]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_configure_accel_bias(struct MipInterfaceState* device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, bias[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, cmdUsed);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// @param[out] bias accelerometer bias in the sensor frame (x,y,z) [g]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_configure_accel_bias(struct MipInterfaceState* device, float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, cmdUsed, MIP_REPLY_DESC_3DM_ACCEL_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &bias[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_configure_accel_bias(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, cmdUsed);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_configure_accel_bias(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, cmdUsed);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_configure_accel_bias(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_ACCEL_BIAS, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GyroBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GyroBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GyroBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GyroBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_GyroBias_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GyroBias_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// @param bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_configure_gyro_bias(struct MipInterfaceState* device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, bias[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, cmdUsed);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// @param[out] bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_configure_gyro_bias(struct MipInterfaceState* device, float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &bias[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_configure_gyro_bias(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, cmdUsed);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_configure_gyro_bias(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, cmdUsed);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_configure_gyro_bias(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_GYRO_BIAS, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_CaptureGyroBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CaptureGyroBias* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->averaging_time_ms);
    
    return offset;
}

size_t extract_MipCmd_3dm_CaptureGyroBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CaptureGyroBias* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->averaging_time_ms);
    
    return offset;
}


size_t insert_MipCmd_3dm_CaptureGyroBias_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CaptureGyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_CaptureGyroBias_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CaptureGyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


/// @brief Samples gyro for a specified time range and writes the averaged result to the Gyro Bias vector in RAM
/// 
/// The device will average the gyro output for the duration of "averaging_time_ms." To store the resulting vector
/// in non-volatile memory, use the Set Gyro Bias command.
/// IMPORTANT: The device must be stationary and experiencing minimum vibration for the duration of "averaging_time_ms"
/// Averaging Time range: 1000 to 30,000
/// @param averaging_time_ms Averaging time [milliseconds]
/// @param[out] bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult capture_gyro_bias(struct MipInterfaceState* device, uint16_t averaging_time_ms, float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_u16(buffer, sizeof(buffer), cmdUsed, averaging_time_ms);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CAPTURE_GYRO_BIAS, buffer, cmdUsed, MIP_REPLY_DESC_3DM_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &bias[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_MagHardIronOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagHardIronOffset* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagHardIronOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagHardIronOffset* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_MagHardIronOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagHardIronOffset_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagHardIronOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagHardIronOffset_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// @param offset hard iron offset in the sensor frame (x,y,z) [Gauss]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_magnetometer_hard_iron_offset(struct MipInterfaceState* device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    for(unsigned int i=0; i < 3; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, cmdUsed);
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// @param[out] offset hard iron offset in the sensor frame (x,y,z) [Gauss]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_magnetometer_hard_iron_offset(struct MipInterfaceState* device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, cmdUsed, MIP_REPLY_DESC_3DM_HARD_IRON_OFFSET_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 3; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &offset[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_magnetometer_hard_iron_offset(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, cmdUsed);
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_magnetometer_hard_iron_offset(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, cmdUsed);
}

/// @brief Configure the user specified magnetometer hard iron offset vector
/// 
/// The values for this offset are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The offset is applied to the scaled magnetometer vector prior to output.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_magnetometer_hard_iron_offset(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_HARD_IRON_OFFSET, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_MagSoftIronMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagSoftIronMatrix* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagSoftIronMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagSoftIronMatrix* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_MagSoftIronMatrix_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagSoftIronMatrix_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagSoftIronMatrix_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagSoftIronMatrix_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// @param offset soft iron matrix [dimensionless]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_magnetometer_soft_iron_matrix(struct MipInterfaceState* device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    for(unsigned int i=0; i < 9; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, offset[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, cmdUsed);
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// @param[out] offset soft iron matrix [dimensionless]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_magnetometer_soft_iron_matrix(struct MipInterfaceState* device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, cmdUsed, MIP_REPLY_DESC_3DM_SOFT_IRON_COMP_MATRIX, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 9; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &offset[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_magnetometer_soft_iron_matrix(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, cmdUsed);
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_magnetometer_soft_iron_matrix(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, cmdUsed);
}

/// @brief Configure the user specified magnetometer soft iron offset matrix
/// 
/// The values for this matrix are determined empirically by external software algorithms
/// based on calibration data taken after the device is installed in its application. These values
/// can be obtained and set by using the LORD "MIP Iron Calibration" application.
/// Alternatively, on some systems, the auto-mag calibration feature may be used to capture these values in-run.
/// The matrix is applied to the scaled magnetometer vector prior to output.
/// 
/// The matrix is in row major order:
/// EQSTART M = \begin{bmatrix} 0 &amp; 1 &amp; 2 \\ 3 &amp; 4 &amp; 5 \\ 6 &amp; 7 &amp; 8 \end{bmatrix} EQEND
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_magnetometer_soft_iron_matrix(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SOFT_IRON_MATRIX, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Sensor2VehicleTransformEuler(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformEuler* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformEuler(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformEuler* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


size_t insert_MipCmd_3dm_Sensor2VehicleTransformEuler_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformEuler_Response* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformEuler_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformEuler_Response* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param roll [radians]
/// @param pitch [radians]
/// @param yaw [radians]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_sensor_to_vehicle_frame_transformation_euler(struct MipInterfaceState* device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, roll);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, pitch);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, yaw);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, cmdUsed);
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param[out] roll [radians]
/// @param[out] pitch [radians]
/// @param[out] yaw [radians]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_sensor_to_vehicle_frame_transformation_euler(struct MipInterfaceState* device, float* roll, float* pitch, float* yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, cmdUsed, MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, roll);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, pitch);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, yaw);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_sensor_to_vehicle_frame_transformation_euler(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, cmdUsed);
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_sensor_to_vehicle_frame_transformation_euler(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, cmdUsed);
}

/// @brief Sets the sensor-to-vehicle frame transformation using Yaw, Pitch, Roll Euler angles.
/// These are the Yaw, Pitch, and Roll mounting angles of the sensor with respect to vehicle frame of reference,
/// and describe the transformation of vectors from the sensor body frame to the vehicle frame.<br/>
/// Note: This is the transformation, the inverse of the rotation defined in our legacy products.<br/>
/// The transformation may be stored in the device as a matrix or quaternion.  When Euler angles are read back from the device, they may not
/// be exactly equal to the Euler angles used to set the transformation, but they are functionally equivalent, such that they result in the same transformation.<br/>
/// <br/><br/>
/// This transformation to the vehicle frame will be applied to the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// Complementary Filter Orientation<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_sensor_to_vehicle_frame_transformation_euler(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Sensor2VehicleTransformQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformQuaternion* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformQuaternion* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    
    return offset;
}


/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param q Unit length quaternion representing transform [w, i, j, k]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_sensor_to_vehicle_frame_transformation_quaternion(struct MipInterfaceState* device, const float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    for(unsigned int i=0; i < 4; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, q[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, cmdUsed);
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param[out] q Unit length quaternion representing transform [w, i, j, k]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_sensor_to_vehicle_frame_transformation_quaternion(struct MipInterfaceState* device, float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, cmdUsed, MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 4; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &q[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_sensor_to_vehicle_frame_transformation_quaternion(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, cmdUsed);
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_sensor_to_vehicle_frame_transformation_quaternion(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, cmdUsed);
}

/// @brief Set the sensor to vehicle frame transformation using unit length quaternion.
/// 
/// Note: This is the transformation, the inverse of the rotation.
/// 
/// This quaternion describes the transformation of vectors from the sensor body frame to the vehicle frame of reference, and satisfies the following relationship:<br/>
/// 
/// EQSTART p^{veh} = q^{-1} p^{sen} q EQEND<br/>
/// 
/// Where:<br/>
/// EQSTART q = (q_w, q_x, q_y, q_z) EQEND is the quaternion desrcribing the transformation. <br/>
/// EQSTART p^{sen} = (0, v^{sen}_x, v^{sen}_y, v^{sen}_z) EQEND and EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame.<br/>
/// EQSTART p^{veh} = (0, v^{veh}_x, v^{veh}_y, v^{veh}_z) EQEND and EQSTART v^{veh} EQEND is a 3-element vector expressed in the vehicle frame.<br/>
/// 
/// The transformation may be stored in the device as a matrix or a quaternion.  When the quaternion is read back from the device, it may not
/// be exactly equal to the quaternion used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_sensor_to_vehicle_frame_transformation_quaternion(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Sensor2VehicleTransformDcm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformDcm* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformDcm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformDcm* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_Sensor2VehicleTransformDcm_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformDcm_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformDcm_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformDcm_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param dcm 3 x 3 direction cosine matrix, stored in row-major order
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_sensor_to_vehicle_frame_transformation_direction_cosine_matrix(struct MipInterfaceState* device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    for(unsigned int i=0; i < 9; i++)
        cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, dcm[i]);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, cmdUsed);
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// @param[out] dcm 3 x 3 direction cosine matrix, stored in row-major order
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_sensor_to_vehicle_frame_transformation_direction_cosine_matrix(struct MipInterfaceState* device, float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, cmdUsed, MIP_REPLY_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        for(unsigned int i=0; i < 9; i++)
            responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, &dcm[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_sensor_to_vehicle_frame_transformation_direction_cosine_matrix(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, cmdUsed);
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_sensor_to_vehicle_frame_transformation_direction_cosine_matrix(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, cmdUsed);
}

/// @brief Set the sensor to vehicle frame transformation using a using a 3 x 3 direction cosine matrix EQSTART M_{ned}^{veh} EQEND, stored in row-major order in a 9-element array.
/// 
/// These angles define the transformation of vectors from the sensor body frame to the fixed vehicle frame, according to:<br/>
/// EQSTART v^{veh} = M_{sen}^{veh} v^{sen} EQEND<br/>
/// 
/// Where:<br/>
/// 
/// EQSTART v^{sen} EQEND is a 3-element vector expressed in the sensor body frame. <br/>
/// EQSTART v^{veh} EQEND is the same 3-element vector expressed in the vehicle frame.  <br/>
/// <br/>
/// The matrix elements are stored is row-major order: EQSTART M_{sen}^{veh} = \begin{bmatrix} M_{11}, M_{12}, M_{13}, M_{21}, M_{22}, M_{23}, M_{31}, M_{32}, M_{33} \end{bmatrix} EQEND
/// The transformation may be stored in the device as a matrix or a quaternion. When EQSTART M_{sen}^{veh} EQEND is read back from the device, it may not
/// be exactly equal to array used to set the transformation, but it is functionally equivalent.<br/>
/// <br/><br/>
/// This transformation affects the following output quantities:<br/><br/>
/// IMU:<br/>
/// Scaled Acceleration<br/>
/// Scaled Gyro<br/>
/// Scaled Magnetometer<br/>
/// Delta Theta<br/>
/// Delta Velocity<br/>
/// <br/><br/>
/// Estimation Filter:<br/>
/// Estimated Orientation, Quaternion<br/>
/// Estimated Orientation, Matrix<br/>
/// Estimated Orientation, Euler Angles<br/>
/// Estimated Linear Acceleration<br/>
/// Estimated Angular Rate<br/>
/// Estimated Gravity Vector<br/>
/// <br/>
/// Changing this setting will force all low-pass filters, the complementary filter, and the estimation filter to reset.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_sensor_to_vehicle_frame_transformation_direction_cosine_matrix(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_ComplementaryFilter(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ComplementaryFilter* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->pitch_roll_enable);
    offset = insert_bool(buffer, bufferSize, offset, self->heading_enable);
    offset = insert_float(buffer, bufferSize, offset, self->pitch_roll_time_constant);
    offset = insert_float(buffer, bufferSize, offset, self->heading_time_constant);
    
    return offset;
}

size_t extract_MipCmd_3dm_ComplementaryFilter(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ComplementaryFilter* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->pitch_roll_enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->heading_enable);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch_roll_time_constant);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_time_constant);
    
    return offset;
}


size_t insert_MipCmd_3dm_ComplementaryFilter_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ComplementaryFilter_Response* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->pitch_roll_enable);
    offset = insert_bool(buffer, bufferSize, offset, self->heading_enable);
    offset = insert_float(buffer, bufferSize, offset, self->pitch_roll_time_constant);
    offset = insert_float(buffer, bufferSize, offset, self->heading_time_constant);
    
    return offset;
}

size_t extract_MipCmd_3dm_ComplementaryFilter_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ComplementaryFilter_Response* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->pitch_roll_enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->heading_enable);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch_roll_time_constant);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_time_constant);
    
    return offset;
}


/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// @param pitch_roll_enable Enable Pitch/Roll corrections
/// @param heading_enable Enable Heading corrections (only available on devices with magnetometer)
/// @param pitch_roll_time_constant Time constant associated with the pitch/roll corrections [s]
/// @param heading_time_constant Time constant associated with the heading corrections [s]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_complementary_filter_settings(struct MipInterfaceState* device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, pitch_roll_enable);
    cmdUsed = insert_bool(buffer, sizeof(buffer), cmdUsed, heading_enable);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, pitch_roll_time_constant);
    cmdUsed = insert_float(buffer, sizeof(buffer), cmdUsed, heading_time_constant);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, cmdUsed);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// @param[out] pitch_roll_enable Enable Pitch/Roll corrections
/// @param[out] heading_enable Enable Heading corrections (only available on devices with magnetometer)
/// @param[out] pitch_roll_time_constant Time constant associated with the pitch/roll corrections [s]
/// @param[out] heading_time_constant Time constant associated with the heading corrections [s]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_complementary_filter_settings(struct MipInterfaceState* device, bool* pitch_roll_enable, bool* heading_enable, float* pitch_roll_time_constant, float* heading_time_constant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, cmdUsed, MIP_REPLY_DESC_3DM_LEGACY_COMP_FILTER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, pitch_roll_enable);
        responseUsed = extract_bool(buffer, sizeof(buffer), responseUsed, heading_enable);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, pitch_roll_time_constant);
        responseUsed = extract_float(buffer, sizeof(buffer), responseUsed, heading_time_constant);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_complementary_filter_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, cmdUsed);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_complementary_filter_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, cmdUsed);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_complementary_filter_settings(struct MipInterfaceState* device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_LEGACY_COMP_FILTER, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_SensorRange(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_SensorRange* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    offset = insert_u8(buffer, bufferSize, offset, self->setting);
    
    return offset;
}

size_t extract_MipCmd_3dm_SensorRange(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_SensorRange* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    offset = extract_u8(buffer, bufferSize, offset, &self->setting);
    
    return offset;
}


size_t insert_MipCmd_3dm_SensorRange_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_SensorRange_Response* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    offset = insert_u8(buffer, bufferSize, offset, self->setting);
    
    return offset;
}

size_t extract_MipCmd_3dm_SensorRange_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_SensorRange_Response* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    offset = extract_u8(buffer, bufferSize, offset, &self->setting);
    
    return offset;
}


/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// @param setting Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult write_sensor_range(struct MipInterfaceState* device, enum MipSensorRangeType sensor, uint8_t setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 1);;
    cmdUsed = insert_MipSensorRangeType(buffer, sizeof(buffer), cmdUsed, sensor);
    cmdUsed = insert_u8(buffer, sizeof(buffer), cmdUsed, setting);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, cmdUsed);
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// @param[out] sensor Which type of sensor will get the new range value.
/// @param[out] setting Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine this value.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult read_sensor_range(struct MipInterfaceState* device, enum MipSensorRangeType sensor, uint8_t* setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 2);;
    cmdUsed = insert_MipSensorRangeType(buffer, sizeof(buffer), cmdUsed, sensor);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, cmdUsed, MIP_REPLY_DESC_3DM_SENSOR_RANGE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipSensorRangeType(buffer, sizeof(buffer), responseUsed, &sensor);
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, setting);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult save_sensor_range(struct MipInterfaceState* device, enum MipSensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 3);;
    cmdUsed = insert_MipSensorRangeType(buffer, sizeof(buffer), cmdUsed, sensor);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, cmdUsed);
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult load_sensor_range(struct MipInterfaceState* device, enum MipSensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 4);;
    cmdUsed = insert_MipSensorRangeType(buffer, sizeof(buffer), cmdUsed, sensor);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, cmdUsed);
}

/// @brief Changes the IMU sensor gain.
/// 
/// This allows you to optimize the range to get the best accuracy and performance
/// while minimizing overrange events.
/// 
/// Use the 3DM Get Calibrated Sensor Ranges (0x0C,0x53) command to determine
/// the appropriate setting value for your application. Using values other than
/// those specified may result in a NACK or inaccurate measurement data.
/// @param sensor Which type of sensor will get the new range value.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult default_sensor_range(struct MipInterfaceState* device, enum MipSensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipFunctionSelector(buffer, sizeof(buffer), cmdUsed, 5);;
    cmdUsed = insert_MipSensorRangeType(buffer, sizeof(buffer), cmdUsed, sensor);
    assert(cmdUsed <= sizeof(buffer));
    
    return MipInterface_runCommand(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_SENSOR_RANGE, buffer, cmdUsed);
}

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_CalibratedSensorRanges(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CalibratedSensorRanges* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    
    return offset;
}

size_t extract_MipCmd_3dm_CalibratedSensorRanges(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CalibratedSensorRanges* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    
    return offset;
}


size_t insert_MipCmd_3dm_CalibratedSensorRanges_Entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CalibratedSensorRanges_Entry* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->setting);
    offset = insert_float(buffer, bufferSize, offset, self->range);
    
    return offset;
}

size_t extract_MipCmd_3dm_CalibratedSensorRanges_Entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CalibratedSensorRanges_Entry* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->setting);
    offset = extract_float(buffer, bufferSize, offset, &self->range);
    
    return offset;
}


size_t insert_MipCmd_3dm_CalibratedSensorRanges_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CalibratedSensorRanges_Response* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    offset = insert_u8(buffer, bufferSize, offset, self->num_ranges);
    assert(self->num_ranges <= 0);
    for(unsigned int i=0; i < self->num_ranges; i++)
        offset = insert_MipCmd_3dm_CalibratedSensorRanges_Entry(buffer, bufferSize, offset, &self->ranges[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_CalibratedSensorRanges_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CalibratedSensorRanges_Response* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    uint8_t num_ranges_local;
    offset = extract_u8(buffer, bufferSize, offset, &num_ranges_local);
    if( num_ranges_local < self->num_ranges )
        self->num_ranges = num_ranges_local;
    for(unsigned int i=0; i < self->num_ranges; i++)
        offset = extract_MipCmd_3dm_CalibratedSensorRanges_Entry(buffer, bufferSize, offset, &self->ranges[i]);
    
    return offset;
}


/// @brief Returns the supported sensor ranges which may be used with the 3DM Sensor Range (0x0C,0x52) command.
/// 
/// The response includes an array of (u8, float) pairs which map each allowed setting
/// to the corresponding maximum range in physical units. See SensorRangeType for units.
/// @param sensor The sensor to query. Cannot be ALL.
/// @param[out] sensor The sensor type from the command.
/// @param[out] num_ranges Number of supported ranges.
/// @param[out] ranges List of possible range settings.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult get_calibrated_sensor_ranges(struct MipInterfaceState* device, enum MipSensorRangeType sensor, uint8_t* num_ranges, struct MipCmd_3dm_CalibratedSensorRanges_Entry* ranges)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    size_t cmdUsed = 0;
    cmdUsed = insert_MipSensorRangeType(buffer, sizeof(buffer), cmdUsed, sensor);
    assert(cmdUsed <= sizeof(buffer));
    
    uint8_t responseLength;
    MipCmdResult result_local = MipInterface_runCommandWithResponse(device, MIP_3DM_COMMAND_DESC_SET, MIP_CMD_DESC_3DM_CALIBRATED_RANGES, buffer, cmdUsed, MIP_REPLY_DESC_3DM_CALIBRATED_RANGES, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        size_t responseUsed = 0;
        responseUsed = extract_MipSensorRangeType(buffer, sizeof(buffer), responseUsed, &sensor);
        uint8_t num_ranges_local;
        responseUsed = extract_u8(buffer, sizeof(buffer), responseUsed, &num_ranges_local);
        if( num_ranges_local < *num_ranges )
            *num_ranges = num_ranges_local;
        for(unsigned int i=0; i < *num_ranges; i++)
            responseUsed = extract_MipCmd_3dm_CalibratedSensorRanges_Entry(buffer, sizeof(buffer), responseUsed, &ranges[i]);
        
        if( responseUsed != responseLength )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
