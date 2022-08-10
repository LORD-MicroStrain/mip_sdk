
#include "commands_3dm.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_3dm {

using ::mscl::insert;
using ::mscl::extract;
using namespace ::mscl::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const NMEAMessageFormat& self)
{
    insert(serializer, self.message_id);
    insert(serializer, self.talker_id);
    insert(serializer, self.source_id);
    insert(serializer, self.decimation);
}

void extract(MipSerializer& serializer, NMEAMessageFormat& self)
{
    extract(serializer, self.message_id);
    extract(serializer, self.talker_id);
    extract(serializer, self.source_id);
    extract(serializer, self.decimation);
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const PollImuMessage& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollImuMessage& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
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
MipCmdResult pollImuMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppress_ack);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_IMU_MESSAGE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const PollGnssMessage& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollGnssMessage& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
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
MipCmdResult pollGnssMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppress_ack);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_GNSS_MESSAGE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const PollFilterMessage& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollFilterMessage& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
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
MipCmdResult pollFilterMessage(C::mip_interface& device, bool suppress_ack, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppress_ack);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_FILTER_MESSAGE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const ImuMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, ImuMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors
/// @param descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeImuMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors
/// @param[out] descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readImuMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_IMU_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        mscl::C::extract_count(&serializer, &num_descriptors, num_descriptors);
        for(unsigned int i=0; i < num_descriptors; i++)
            extract(serializer, descriptors[i]);
        
        if( !!!serializer )
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
MipCmdResult saveImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the IMU data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GpsMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, GpsMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors
/// @param descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeGpsMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors
/// @param[out] descriptors Descriptor format list.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readGpsMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_GNSS_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        mscl::C::extract_count(&serializer, &num_descriptors, num_descriptors);
        for(unsigned int i=0; i < num_descriptors; i++)
            extract(serializer, descriptors[i]);
        
        if( !!!serializer )
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
MipCmdResult saveGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the GNSS data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const FilterMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, FilterMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param num_descriptors Number of descriptors (limited by payload size)
/// @param descriptors 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeFilterMessageFormat(C::mip_interface& device, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param[out] num_descriptors Number of descriptors (limited by payload size)
/// @param[out] descriptors 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readFilterMessageFormat(C::mip_interface& device, uint8_t& num_descriptors, MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_FILTER_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        mscl::C::extract_count(&serializer, &num_descriptors, num_descriptors);
        for(unsigned int i=0; i < num_descriptors; i++)
            extract(serializer, descriptors[i]);
        
        if( !!!serializer )
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
MipCmdResult saveFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format of the Estimation Filter data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the base rate for the IMU data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the IMU Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult imuGetBaseRate(C::mip_interface& device, uint16_t& rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMU_BASE_RATE, NULL, 0, REPLY_IMU_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, rate);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the base rate for the GNSS data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the GNSS Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult gpsGetBaseRate(C::mip_interface& device, uint16_t& rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_GNSS_BASE_RATE, NULL, 0, REPLY_GNSS_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, rate);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

/// @brief Get the base rate for the Estimation Filter data in Hz
/// 
/// This is the fastest rate for this type of data available on the device.
/// This is used in conjunction with the Estimation Filter Message Format Command to set streaming data at a specified rate.
/// @param[out] rate [hz]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult filterGetBaseRate(C::mip_interface& device, uint16_t& rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_FILTER_BASE_RATE, NULL, 0, REPLY_FILTER_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, rate);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const PollData& self)
{
    insert(serializer, self.desc_set);
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollData& self)
{
    extract(serializer, self.desc_set);
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
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
MipCmdResult pollData(C::mip_interface& device, uint8_t desc_set, bool suppress_ack, uint8_t num_descriptors, const uint8_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, desc_set);
    insert(serializer, suppress_ack);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_DATA, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GetBaseRate& self)
{
    insert(serializer, self.desc_set);
}

void extract(MipSerializer& serializer, GetBaseRate& self)
{
    extract(serializer, self.desc_set);
}

/// @brief Get the base rate for the specified descriptor set in Hz.
/// 
/// @param desc_set This is the data descriptor set. It must be a supported descriptor.
/// @param[out] desc_set Echoes the parameter in the command.
/// @param[out] rate Base rate in Hz (0 = variable, unknown, or user-defined rate.  Data will be sent when received).
/// 
/// @returns MipCmdResult
/// 
MipCmdResult getBaseRate(C::mip_interface& device, uint8_t desc_set, uint16_t& rate)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, desc_set);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_BASE_RATE, buffer, serializer.offset, REPLY_BASE_RATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, desc_set);
        extract(serializer, rate);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const MessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.desc_set);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, MessageFormat& self)
{
    extract(serializer, self.function);
    extract(serializer, self.desc_set);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
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
MipCmdResult writeMessageFormat(C::mip_interface& device, uint8_t desc_set, uint8_t num_descriptors, const MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, desc_set);
    insert(serializer, num_descriptors);
    for(unsigned int i=0; i < num_descriptors; i++)
        insert(serializer, descriptors[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
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
MipCmdResult readMessageFormat(C::mip_interface& device, uint8_t desc_set, uint8_t& num_descriptors, MipDescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, desc_set);
        mscl::C::extract_count(&serializer, &num_descriptors, num_descriptors);
        for(unsigned int i=0; i < num_descriptors; i++)
            extract(serializer, descriptors[i]);
        
        if( !!!serializer )
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
MipCmdResult saveMessageFormat(C::mip_interface& device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadMessageFormat(C::mip_interface& device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the format for a given data packet.
/// 
/// The resulting data messages will maintain the order of descriptors sent in the command.
/// @param desc_set Data descriptor set. Must be supported. When function is SAVE, LOAD, or DEFAULT, can be 0 to apply to all descriptor sets.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultMessageFormat(C::mip_interface& device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const NmeaPollData& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.count);
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
}

void extract(MipSerializer& serializer, NmeaPollData& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.count, self.count);
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
}

/// @brief Poll the device for a NMEA message with the specified format.
/// 
/// This function polls for a NMEA message using the provided format.
/// If the format is not provided, the device will attempt to use the
/// stored format (set with the Set NMEA Message Format command.) If no format is provided
/// and there is no stored format, the device will respond with a NACK. The reply packet contains
/// an ACK/NACK field. The polled data packet is sent separately as normal NMEA messages.
/// @param suppress_ack Suppress the usual ACK/NACK reply.
/// @param count Number of format entries (limited by payload size)
/// @param format_entries List of format entries.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult nmeaPollData(C::mip_interface& device, bool suppress_ack, uint8_t count, const NMEAMessageFormat* format_entries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppress_ack);
    insert(serializer, count);
    for(unsigned int i=0; i < count; i++)
        insert(serializer, format_entries[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_NMEA_MESSAGE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const NmeaMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.count);
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
}

void extract(MipSerializer& serializer, NmeaMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.count, self.count);
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// @param count Number of format entries (limited by payload size)
/// @param format_entries List of format entries.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const NMEAMessageFormat* format_entries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, count);
    for(unsigned int i=0; i < count; i++)
        insert(serializer, format_entries[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// @param[out] count Number of format entries (limited by payload size)
/// @param[out] format_entries List of format entries.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readNmeaMessageFormat(C::mip_interface& device, uint8_t& count, NMEAMessageFormat* format_entries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_NMEA_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        mscl::C::extract_count(&serializer, &count, count);
        for(unsigned int i=0; i < count; i++)
            extract(serializer, format_entries[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult saveNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

/// @brief Set, read, or save the NMEA message format.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const DeviceSettings& self)
{
    insert(serializer, self.function);
}

void extract(MipSerializer& serializer, DeviceSettings& self)
{
    extract(serializer, self.function);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult saveDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}

/// @brief Save, Load, or Reset to Default the values for all device settings.
/// 
/// When a save current settings command is issued, a brief data disturbance may occur while all settings are written to non-volatile memory.
/// 
/// This command should have a long timeout as it may take up to 1 second to complete.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const UartBaudrate& self)
{
    insert(serializer, self.function);
    insert(serializer, self.baud);
}

void extract(MipSerializer& serializer, UartBaudrate& self)
{
    extract(serializer, self.function);
    extract(serializer, self.baud);
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
MipCmdResult writeUartBaudrate(C::mip_interface& device, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, baud);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
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
MipCmdResult readUartBaudrate(C::mip_interface& device, uint32_t& baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset, REPLY_UART_BAUDRATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, baud);
        
        if( !!!serializer )
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
MipCmdResult saveUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
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
MipCmdResult loadUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
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
MipCmdResult defaultUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const FactoryStreaming& self)
{
    insert(serializer, self.action);
    insert(serializer, self.reserved);
}

void extract(MipSerializer& serializer, FactoryStreaming& self)
{
    extract(serializer, self.action);
    extract(serializer, self.reserved);
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
MipCmdResult factoryStreaming(C::mip_interface& device, FactoryStreaming::Action action, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, action);
    insert(serializer, reserved);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONFIGURE_FACTORY_STREAMING, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const DatastreamControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.desc_set);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, DatastreamControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.desc_set);
    extract(serializer, self.enable);
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
MipCmdResult writeDatastreamControl(C::mip_interface& device, uint8_t desc_set, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, desc_set);
    insert(serializer, enable);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
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
MipCmdResult readDatastreamControl(C::mip_interface& device, uint8_t desc_set, bool& enabled)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset, REPLY_DATASTREAM_ENABLE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, desc_set);
        extract(serializer, enabled);
        
        if( !!!serializer )
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
MipCmdResult saveDatastreamControl(C::mip_interface& device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
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
MipCmdResult loadDatastreamControl(C::mip_interface& device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
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
MipCmdResult defaultDatastreamControl(C::mip_interface& device, uint8_t desc_set)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, desc_set);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GnssSbasSettings& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable_sbas);
    insert(serializer, self.sbas_options);
    insert(serializer, self.num_included_prns);
    for(unsigned int i=0; i < self.num_included_prns; i++)
        insert(serializer, self.included_prns[i]);
}

void extract(MipSerializer& serializer, GnssSbasSettings& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable_sbas);
    extract(serializer, self.sbas_options);
    mscl::C::extract_count(&serializer, &self.num_included_prns, self.num_included_prns);
    for(unsigned int i=0; i < self.num_included_prns; i++)
        extract(serializer, self.included_prns[i]);
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
MipCmdResult writeGnssSbasSettings(C::mip_interface& device, uint8_t enable_sbas, GnssSbasSettings::SBASOptions sbas_options, uint8_t num_included_prns, const uint16_t* included_prns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, enable_sbas);
    insert(serializer, sbas_options);
    insert(serializer, num_included_prns);
    for(unsigned int i=0; i < num_included_prns; i++)
        insert(serializer, included_prns[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
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
MipCmdResult readGnssSbasSettings(C::mip_interface& device, uint8_t& enable_sbas, GnssSbasSettings::SBASOptions& sbas_options, uint8_t& num_included_prns, uint16_t* included_prns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset, REPLY_GNSS_SBAS_SETTINGS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, enable_sbas);
        extract(serializer, sbas_options);
        mscl::C::extract_count(&serializer, &num_included_prns, num_included_prns);
        for(unsigned int i=0; i < num_included_prns; i++)
            extract(serializer, included_prns[i]);
        
        if( !!!serializer )
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
MipCmdResult saveGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}

/// @brief Configure the SBAS subsystem
/// 
/// 
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GnssTimeAssistance& self)
{
    insert(serializer, self.function);
    insert(serializer, self.tow);
    insert(serializer, self.week_number);
    insert(serializer, self.accuracy);
}

void extract(MipSerializer& serializer, GnssTimeAssistance& self)
{
    extract(serializer, self.function);
    extract(serializer, self.tow);
    extract(serializer, self.week_number);
    extract(serializer, self.accuracy);
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
MipCmdResult writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t week_number, float accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, tow);
    insert(serializer, week_number);
    insert(serializer, accuracy);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, serializer.offset);
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
MipCmdResult readGnssTimeAssistance(C::mip_interface& device, double& tow, uint16_t& week_number, float& accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, serializer.offset, REPLY_GNSS_TIME_ASSISTANCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, tow);
        extract(serializer, week_number);
        extract(serializer, accuracy);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const AdvLowpassFilter& self)
{
    insert(serializer, self.function);
    insert(serializer, self.target_descriptor);
    insert(serializer, self.enable);
    insert(serializer, self.manual);
    insert(serializer, self.frequency);
    insert(serializer, self.reserved);
}

void extract(MipSerializer& serializer, AdvLowpassFilter& self)
{
    extract(serializer, self.function);
    extract(serializer, self.target_descriptor);
    extract(serializer, self.enable);
    extract(serializer, self.manual);
    extract(serializer, self.frequency);
    extract(serializer, self.reserved);
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
MipCmdResult writeAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, target_descriptor);
    insert(serializer, enable);
    insert(serializer, manual);
    insert(serializer, frequency);
    insert(serializer, reserved);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
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
MipCmdResult readAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor, bool& enable, bool& manual, uint16_t& frequency, uint8_t& reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, target_descriptor);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset, REPLY_ADVANCED_DATA_FILTER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, target_descriptor);
        extract(serializer, enable);
        extract(serializer, manual);
        extract(serializer, frequency);
        extract(serializer, reserved);
        
        if( !!!serializer )
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
MipCmdResult saveAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, target_descriptor);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
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
MipCmdResult loadAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, target_descriptor);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
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
MipCmdResult defaultAdvLowpassFilter(C::mip_interface& device, uint8_t target_descriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, target_descriptor);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const PpsSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
}

void extract(MipSerializer& serializer, PpsSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// @param source 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writePpsSource(C::mip_interface& device, PpsSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, source);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// @param[out] source 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readPpsSource(C::mip_interface& device, PpsSource::Source& source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset, REPLY_PPS_SOURCE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, source);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult savePpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}

/// @brief Controls the Pulse Per Second (PPS) source.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GpioConfig& self)
{
    insert(serializer, self.function);
    insert(serializer, self.pin);
    insert(serializer, self.feature);
    insert(serializer, self.behavior);
    insert(serializer, self.pin_mode);
}

void extract(MipSerializer& serializer, GpioConfig& self)
{
    extract(serializer, self.function);
    extract(serializer, self.pin);
    extract(serializer, self.feature);
    extract(serializer, self.behavior);
    extract(serializer, self.pin_mode);
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
MipCmdResult writeGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature feature, GpioConfig::Behavior behavior, GpioConfig::PinMode pin_mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, pin);
    insert(serializer, feature);
    insert(serializer, behavior);
    insert(serializer, pin_mode);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
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
MipCmdResult readGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature& feature, GpioConfig::Behavior& behavior, GpioConfig::PinMode& pin_mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, pin);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset, REPLY_GPIO_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, pin);
        extract(serializer, feature);
        extract(serializer, behavior);
        extract(serializer, pin_mode);
        
        if( !!!serializer )
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
MipCmdResult saveGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, pin);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
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
MipCmdResult loadGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, pin);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
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
MipCmdResult defaultGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, pin);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GpioState& self)
{
    insert(serializer, self.function);
    insert(serializer, self.pin);
    insert(serializer, self.state);
}

void extract(MipSerializer& serializer, GpioState& self)
{
    extract(serializer, self.function);
    extract(serializer, self.pin);
    extract(serializer, self.state);
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
MipCmdResult writeGpioState(C::mip_interface& device, uint8_t pin, bool state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, pin);
    insert(serializer, state);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, serializer.offset);
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
MipCmdResult readGpioState(C::mip_interface& device, uint8_t pin, bool& state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, pin);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, serializer.offset, REPLY_GPIO_STATE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, pin);
        extract(serializer, state);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const Odometer& self)
{
    insert(serializer, self.function);
    insert(serializer, self.mode);
    insert(serializer, self.scaling);
    insert(serializer, self.uncertainty);
}

void extract(MipSerializer& serializer, Odometer& self)
{
    extract(serializer, self.function);
    extract(serializer, self.mode);
    extract(serializer, self.scaling);
    extract(serializer, self.uncertainty);
}

/// @brief Configures the hardware odometer interface.
/// 
/// @param mode Mode setting.
/// @param scaling Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
/// @param uncertainty Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeOdometer(C::mip_interface& device, Odometer::Mode mode, float scaling, float uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, mode);
    insert(serializer, scaling);
    insert(serializer, uncertainty);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the hardware odometer interface.
/// 
/// @param[out] mode Mode setting.
/// @param[out] scaling Encoder pulses per meter of distance traveled [pulses/m]. Distance traveled is computed using the formula d = p / N * 2R * pi, where d is distance, p is the number of pulses received, N is the encoder resolution, and R is the wheel radius. By simplifying all of the parameters into one, the formula d = p / S is obtained, where s is the odometer scaling factor passed to this command. S is equivalent to N / (2R * pi) and has units of pulses / meter. N is in units of "A" pulses per revolution and R is in meters. Make this value negative if the odometer is mounted so that it rotates backwards.
/// @param[out] uncertainty Uncertainty in encoder counts to distance translation (1-sigma value) [m/m].
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readOdometer(C::mip_interface& device, Odometer::Mode& mode, float& scaling, float& uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset, REPLY_ODOMETER_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, mode);
        extract(serializer, scaling);
        extract(serializer, uncertainty);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult saveOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures the hardware odometer interface.
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GetEventSupport& self)
{
    insert(serializer, self.query);
}

void extract(MipSerializer& serializer, GetEventSupport& self)
{
    extract(serializer, self.query);
}

void insert(MipSerializer& serializer, const GetEventSupport::Info& self)
{
    insert(serializer, self.type);
    insert(serializer, self.count);
}

void extract(MipSerializer& serializer, GetEventSupport::Info& self)
{
    extract(serializer, self.type);
    extract(serializer, self.count);
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
MipCmdResult getEventSupport(C::mip_interface& device, GetEventSupport::Query query, uint8_t& max_instances, uint8_t& num_entries, GetEventSupport::Info* entries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, query);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_SUPPORT, buffer, serializer.offset, REPLY_EVENT_SUPPORT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, query);
        extract(serializer, max_instances);
        mscl::C::extract_count(&serializer, &num_entries, num_entries);
        for(unsigned int i=0; i < num_entries; i++)
            extract(serializer, entries[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const EventControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.instance);
    insert(serializer, self.mode);
}

void extract(MipSerializer& serializer, EventControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.instance);
    extract(serializer, self.mode);
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
MipCmdResult writeEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, instance);
    insert(serializer, mode);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
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
MipCmdResult readEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode& mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, instance);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset, REPLY_EVENT_CONTROL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, instance);
        extract(serializer, mode);
        
        if( !!!serializer )
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
MipCmdResult saveEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
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
MipCmdResult loadEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
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
MipCmdResult defaultEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GetEventTriggerStatus& self)
{
    insert(serializer, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
}

void extract(MipSerializer& serializer, GetEventTriggerStatus& self)
{
    mscl::C::extract_count(&serializer, &self.requested_count, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
}

void insert(MipSerializer& serializer, const GetEventTriggerStatus::Entry& self)
{
    insert(serializer, self.type);
    insert(serializer, self.status);
}

void extract(MipSerializer& serializer, GetEventTriggerStatus::Entry& self)
{
    extract(serializer, self.type);
    extract(serializer, self.status);
}

MipCmdResult getEventTriggerStatus(C::mip_interface& device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t& count, GetEventTriggerStatus::Entry* triggers)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, requested_count);
    for(unsigned int i=0; i < requested_count; i++)
        insert(serializer, requested_instances[i]);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_STATUS, buffer, serializer.offset, REPLY_EVENT_TRIGGER_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        mscl::C::extract_count(&serializer, &count, count);
        for(unsigned int i=0; i < count; i++)
            extract(serializer, triggers[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const GetEventActionStatus& self)
{
    insert(serializer, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
}

void extract(MipSerializer& serializer, GetEventActionStatus& self)
{
    mscl::C::extract_count(&serializer, &self.requested_count, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
}

void insert(MipSerializer& serializer, const GetEventActionStatus::Entry& self)
{
    insert(serializer, self.action_type);
    insert(serializer, self.trigger_id);
}

void extract(MipSerializer& serializer, GetEventActionStatus::Entry& self)
{
    extract(serializer, self.action_type);
    extract(serializer, self.trigger_id);
}

MipCmdResult getEventActionStatus(C::mip_interface& device, uint8_t requested_count, const uint8_t* requested_instances, uint8_t& count, GetEventActionStatus::Entry* actions)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, requested_count);
    for(unsigned int i=0; i < requested_count; i++)
        insert(serializer, requested_instances[i]);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_STATUS, buffer, serializer.offset, REPLY_EVENT_ACTION_STATUS, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        mscl::C::extract_count(&serializer, &count, count);
        for(unsigned int i=0; i < count; i++)
            extract(serializer, actions[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const EventTrigger& self)
{
    insert(serializer, self.function);
    insert(serializer, self.instance);
    insert(serializer, self.type);
    if( self.type == EventTrigger::Type::GPIO )
        insert(serializer, self.parameters.gpio);
    if( self.type == EventTrigger::Type::THRESHOLD )
        insert(serializer, self.parameters.threshold);
    if( self.type == EventTrigger::Type::COMBINATION )
        insert(serializer, self.parameters.combination);
}

void extract(MipSerializer& serializer, EventTrigger& self)
{
    extract(serializer, self.function);
    extract(serializer, self.instance);
    extract(serializer, self.type);
    if( self.type == EventTrigger::Type::GPIO )
        extract(serializer, self.parameters.gpio);
    if( self.type == EventTrigger::Type::THRESHOLD )
        extract(serializer, self.parameters.threshold);
    if( self.type == EventTrigger::Type::COMBINATION )
        extract(serializer, self.parameters.combination);
}

void insert(MipSerializer& serializer, const EventTrigger::GpioParams& self)
{
    insert(serializer, self.pin);
    insert(serializer, self.mode);
}

void extract(MipSerializer& serializer, EventTrigger::GpioParams& self)
{
    extract(serializer, self.pin);
    extract(serializer, self.mode);
}

void insert(MipSerializer& serializer, const EventTrigger::ThresholdParams& self)
{
    insert(serializer, self.desc_set);
    insert(serializer, self.field_desc);
    insert(serializer, self.param_id);
    insert(serializer, self.type);
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
        insert(serializer, self.low_thres);
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
        insert(serializer, self.int_thres);
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
        insert(serializer, self.high_thres);
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
        insert(serializer, self.interval);
}

void extract(MipSerializer& serializer, EventTrigger::ThresholdParams& self)
{
    extract(serializer, self.desc_set);
    extract(serializer, self.field_desc);
    extract(serializer, self.param_id);
    extract(serializer, self.type);
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
        extract(serializer, self.low_thres);
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
        extract(serializer, self.int_thres);
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
        extract(serializer, self.high_thres);
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
        extract(serializer, self.interval);
}

void insert(MipSerializer& serializer, const EventTrigger::CombinationParams& self)
{
    insert(serializer, self.logic_table);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.input_triggers[i]);
}

void extract(MipSerializer& serializer, EventTrigger::CombinationParams& self)
{
    extract(serializer, self.logic_table);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.input_triggers[i]);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param type Type of trigger to configure.
/// @param parameters 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type type, const EventTrigger::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, instance);
    insert(serializer, type);
    if( type == EventTrigger::Type::GPIO )
        insert(serializer, parameters.gpio);
    if( type == EventTrigger::Type::THRESHOLD )
        insert(serializer, parameters.threshold);
    if( type == EventTrigger::Type::COMBINATION )
        insert(serializer, parameters.combination);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] type Type of trigger to configure.
/// @param[out] parameters 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type& type, EventTrigger::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, instance);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset, REPLY_EVENT_TRIGGER_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, instance);
        extract(serializer, type);
        if( type == EventTrigger::Type::GPIO )
            extract(serializer, parameters.gpio);
        if( type == EventTrigger::Type::THRESHOLD )
            extract(serializer, parameters.threshold);
        if( type == EventTrigger::Type::COMBINATION )
            extract(serializer, parameters.combination);
        
        if( !!!serializer )
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
MipCmdResult saveEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event triggers.
/// 
/// @param instance Trigger number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const EventAction& self)
{
    insert(serializer, self.function);
    insert(serializer, self.instance);
    insert(serializer, self.trigger);
    insert(serializer, self.type);
    if( self.type == EventAction::Type::GPIO )
        insert(serializer, self.parameters.gpio);
    if( self.type == EventAction::Type::MESSAGE )
        insert(serializer, self.parameters.message);
}

void extract(MipSerializer& serializer, EventAction& self)
{
    extract(serializer, self.function);
    extract(serializer, self.instance);
    extract(serializer, self.trigger);
    extract(serializer, self.type);
    if( self.type == EventAction::Type::GPIO )
        extract(serializer, self.parameters.gpio);
    if( self.type == EventAction::Type::MESSAGE )
        extract(serializer, self.parameters.message);
}

void insert(MipSerializer& serializer, const EventAction::GpioParams& self)
{
    insert(serializer, self.pin);
    insert(serializer, self.mode);
}

void extract(MipSerializer& serializer, EventAction::GpioParams& self)
{
    extract(serializer, self.pin);
    extract(serializer, self.mode);
}

void insert(MipSerializer& serializer, const EventAction::MessageParams& self)
{
    insert(serializer, self.desc_set);
    insert(serializer, self.decimation);
    insert(serializer, self.num_fields);
    for(unsigned int i=0; i < self.num_fields; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, EventAction::MessageParams& self)
{
    extract(serializer, self.desc_set);
    extract(serializer, self.decimation);
    mscl::C::extract_count(&serializer, &self.num_fields, self.num_fields);
    for(unsigned int i=0; i < self.num_fields; i++)
        extract(serializer, self.descriptors[i]);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param trigger Trigger ID number.
/// @param type Type of action to configure.
/// @param parameters 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, EventAction::Type type, const EventAction::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, instance);
    insert(serializer, trigger);
    insert(serializer, type);
    if( type == EventAction::Type::GPIO )
        insert(serializer, parameters.gpio);
    if( type == EventAction::Type::MESSAGE )
        insert(serializer, parameters.message);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// @param[out] trigger Trigger ID number.
/// @param[out] type Type of action to configure.
/// @param[out] parameters 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readEventAction(C::mip_interface& device, uint8_t instance, uint8_t& trigger, EventAction::Type& type, EventAction::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, instance);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset, REPLY_EVENT_ACTION_CONFIG, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, instance);
        extract(serializer, trigger);
        extract(serializer, type);
        if( type == EventAction::Type::GPIO )
            extract(serializer, parameters.gpio);
        if( type == EventAction::Type::MESSAGE )
            extract(serializer, parameters.message);
        
        if( !!!serializer )
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
MipCmdResult saveEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

/// @brief Configures various types of event actions.
/// 
/// @param instance Action number. When function is SAVE, LOAD, or DEFAULT, this can be 0 to apply to all instances.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, instance);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const AccelBias& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
}

void extract(MipSerializer& serializer, AccelBias& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// @param bias accelerometer bias in the sensor frame (x,y,z) [g]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeAccelBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, bias[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// @param[out] bias accelerometer bias in the sensor frame (x,y,z) [g]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readAccelBias(C::mip_interface& device, float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset, REPLY_ACCEL_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, bias[i]);
        
        if( !!!serializer )
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
MipCmdResult saveAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified accelerometer bias
/// 
/// The user specified bias is subtracted from the calibrated accelerometer output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const GyroBias& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
}

void extract(MipSerializer& serializer, GyroBias& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// @param bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeGyroBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, bias[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// @param[out] bias gyro bias in the sensor frame (x,y,z) [radians/second]
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readGyroBias(C::mip_interface& device, float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset, REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, bias[i]);
        
        if( !!!serializer )
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
MipCmdResult saveGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}

/// @brief Configures the user specified gyroscope bias
/// 
/// The user specified bias is subtracted from the calibrated angular rate output.  Value is input in the sensor frame.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const CaptureGyroBias& self)
{
    insert(serializer, self.averaging_time_ms);
}

void extract(MipSerializer& serializer, CaptureGyroBias& self)
{
    extract(serializer, self.averaging_time_ms);
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
MipCmdResult captureGyroBias(C::mip_interface& device, uint16_t averaging_time_ms, float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, averaging_time_ms);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CAPTURE_GYRO_BIAS, buffer, serializer.offset, REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, bias[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

void insert(MipSerializer& serializer, const MagHardIronOffset& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
}

void extract(MipSerializer& serializer, MagHardIronOffset& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
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
MipCmdResult writeMagHardIronOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
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
MipCmdResult readMagHardIronOffset(C::mip_interface& device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset, REPLY_HARD_IRON_OFFSET_VECTOR, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, offset[i]);
        
        if( !!!serializer )
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
MipCmdResult saveMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
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
MipCmdResult loadMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
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
MipCmdResult defaultMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const MagSoftIronMatrix& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.offset[i]);
}

void extract(MipSerializer& serializer, MagSoftIronMatrix& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.offset[i]);
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
MipCmdResult writeMagSoftIronMatrix(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, offset[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
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
MipCmdResult readMagSoftIronMatrix(C::mip_interface& device, float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset, REPLY_SOFT_IRON_COMP_MATRIX, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, offset[i]);
        
        if( !!!serializer )
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
MipCmdResult saveMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
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
MipCmdResult loadMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
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
MipCmdResult defaultMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const Sensor2VehicleTransformEuler& self)
{
    insert(serializer, self.function);
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
}

void extract(MipSerializer& serializer, Sensor2VehicleTransformEuler& self)
{
    extract(serializer, self.function);
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
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
MipCmdResult writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, roll);
    insert(serializer, pitch);
    insert(serializer, yaw);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
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
MipCmdResult readSensor2VehicleTransformEuler(C::mip_interface& device, float& roll, float& pitch, float& yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, roll);
        extract(serializer, pitch);
        extract(serializer, yaw);
        
        if( !!!serializer )
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
MipCmdResult saveSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
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
MipCmdResult loadSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
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
MipCmdResult defaultSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const Sensor2VehicleTransformQuaternion& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
}

void extract(MipSerializer& serializer, Sensor2VehicleTransformQuaternion& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
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
MipCmdResult writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, q[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
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
MipCmdResult readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, q[i]);
        
        if( !!!serializer )
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
MipCmdResult saveSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
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
MipCmdResult loadSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
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
MipCmdResult defaultSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const Sensor2VehicleTransformDcm& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
}

void extract(MipSerializer& serializer, Sensor2VehicleTransformDcm& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
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
MipCmdResult writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
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
MipCmdResult readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, dcm[i]);
        
        if( !!!serializer )
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
MipCmdResult saveSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
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
MipCmdResult loadSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
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
MipCmdResult defaultSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const ComplementaryFilter& self)
{
    insert(serializer, self.function);
    insert(serializer, self.pitch_roll_enable);
    insert(serializer, self.heading_enable);
    insert(serializer, self.pitch_roll_time_constant);
    insert(serializer, self.heading_time_constant);
}

void extract(MipSerializer& serializer, ComplementaryFilter& self)
{
    extract(serializer, self.function);
    extract(serializer, self.pitch_roll_enable);
    extract(serializer, self.heading_enable);
    extract(serializer, self.pitch_roll_time_constant);
    extract(serializer, self.heading_time_constant);
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
MipCmdResult writeComplementaryFilter(C::mip_interface& device, bool pitch_roll_enable, bool heading_enable, float pitch_roll_time_constant, float heading_time_constant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, pitch_roll_enable);
    insert(serializer, heading_enable);
    insert(serializer, pitch_roll_time_constant);
    insert(serializer, heading_time_constant);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
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
MipCmdResult readComplementaryFilter(C::mip_interface& device, bool& pitch_roll_enable, bool& heading_enable, float& pitch_roll_time_constant, float& heading_time_constant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset, REPLY_LEGACY_COMP_FILTER, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, pitch_roll_enable);
        extract(serializer, heading_enable);
        extract(serializer, pitch_roll_time_constant);
        extract(serializer, heading_time_constant);
        
        if( !!!serializer )
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
MipCmdResult saveComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult loadComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}

/// @brief Configure the settings for the complementary filter which produces the following (0x80) descriptor set values: attitude matrix (0x80,09), quaternion (0x80,0A), and  Euler angle (0x80,0C) outputs.
/// 
/// The filter can be configured to correct for pitch and roll using the accelerometer (with the assumption that linear acceleration is minimal),
/// and to correct for heading using the magnetomer (with the assumption that the local magnetic field is dominated by the Earth's own magnetic field).
/// Pitch/roll and heading corrections each have their own configurable time constants, with a valid range of 1-1000 seconds. The default time constant is 10 seconds.
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const SensorRange& self)
{
    insert(serializer, self.function);
    insert(serializer, self.sensor);
    insert(serializer, self.setting);
}

void extract(MipSerializer& serializer, SensorRange& self)
{
    extract(serializer, self.function);
    extract(serializer, self.sensor);
    extract(serializer, self.setting);
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
MipCmdResult writeSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, sensor);
    insert(serializer, setting);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
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
MipCmdResult readSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t& setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    insert(serializer, sensor);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset, REPLY_SENSOR_RANGE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, sensor);
        extract(serializer, setting);
        
        if( !!!serializer )
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
MipCmdResult saveSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::SAVE);
    insert(serializer, sensor);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
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
MipCmdResult loadSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::LOAD);
    insert(serializer, sensor);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
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
MipCmdResult defaultSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    insert(serializer, sensor);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
}

void insert(MipSerializer& serializer, const CalibratedSensorRanges& self)
{
    insert(serializer, self.sensor);
}

void extract(MipSerializer& serializer, CalibratedSensorRanges& self)
{
    extract(serializer, self.sensor);
}

void insert(MipSerializer& serializer, const CalibratedSensorRanges::Entry& self)
{
    insert(serializer, self.setting);
    insert(serializer, self.range);
}

void extract(MipSerializer& serializer, CalibratedSensorRanges::Entry& self)
{
    extract(serializer, self.setting);
    extract(serializer, self.range);
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
MipCmdResult calibratedSensorRanges(C::mip_interface& device, SensorRangeType sensor, uint8_t& num_ranges, CalibratedSensorRanges::Entry* ranges)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, sensor);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CALIBRATED_RANGES, buffer, serializer.offset, REPLY_CALIBRATED_RANGES, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, sensor);
        mscl::C::extract_count(&serializer, &num_ranges, num_ranges);
        for(unsigned int i=0; i < num_ranges; i++)
            extract(serializer, ranges[i]);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}


} // namespace commands_3dm
} // namespace mscl

