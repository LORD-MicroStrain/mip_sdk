
#include "commands_3dm.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_3dm {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const NMEAMessageFormat& self)
{
    insert(serializer, self.message_id);
    
    insert(serializer, self.talker_id);
    
    insert(serializer, self.source_id);
    
    insert(serializer, self.decimation);
    
}
void extract(Serializer& serializer, NMEAMessageFormat& self)
{
    extract(serializer, self.message_id);
    
    extract(serializer, self.talker_id);
    
    extract(serializer, self.source_id);
    
    extract(serializer, self.decimation);
    
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const PollImuMessage& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollImuMessage& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult pollImuMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_IMU_MESSAGE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const PollGnssMessage& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollGnssMessage& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult pollGnssMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_GNSS_MESSAGE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const PollFilterMessage& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollFilterMessage& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult pollFilterMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_FILTER_MESSAGE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ImuMessageFormat& self)
{
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, ImuMessageFormat& self)
{
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult writeImuMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult readImuMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_IMU_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut);
        assert(numDescriptorsOut);
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult loadImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult defaultImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GpsMessageFormat& self)
{
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, GpsMessageFormat& self)
{
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult writeGpsMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult readGpsMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_GNSS_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut);
        assert(numDescriptorsOut);
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult loadGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult defaultGpsMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const FilterMessageFormat& self)
{
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, FilterMessageFormat& self)
{
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult writeFilterMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult readFilterMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_FILTER_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut);
        assert(numDescriptorsOut);
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult loadFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult defaultFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

CmdResult imuGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMU_BASE_RATE, NULL, 0, REPLY_IMU_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

CmdResult gpsGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_GNSS_BASE_RATE, NULL, 0, REPLY_GNSS_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

CmdResult filterGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_FILTER_BASE_RATE, NULL, 0, REPLY_FILTER_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const PollData& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, PollData& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult pollData(C::mip_interface& device, uint8_t descSet, bool suppressAck, uint8_t numDescriptors, const uint8_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, descSet);
    
    insert(serializer, suppressAck);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_DATA, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GetBaseRate& self)
{
    insert(serializer, self.desc_set);
    
}
void extract(Serializer& serializer, GetBaseRate& self)
{
    extract(serializer, self.desc_set);
    
}

CmdResult getBaseRate(C::mip_interface& device, uint8_t descSet, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_BASE_RATE, buffer, serializer.offset, REPLY_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        assert(rateOut);
        extract(deserializer, *rateOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const MessageFormat& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.num_descriptors);
    
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, MessageFormat& self)
{
    extract(serializer, self.desc_set);
    
    C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult writeMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, descSet);
    
    insert(serializer, numDescriptors);
    
    assert(descriptors);
    for(unsigned int i=0; i < numDescriptors; i++)
        insert(serializer, descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult readMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        C::extract_count(&deserializer, numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut);
        assert(numDescriptorsOut);
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            extract(deserializer, descriptorsOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult loadMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult defaultMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const NmeaPollData& self)
{
    insert(serializer, self.suppress_ack);
    
    insert(serializer, self.count);
    
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
    
}
void extract(Serializer& serializer, NmeaPollData& self)
{
    extract(serializer, self.suppress_ack);
    
    C::extract_count(&serializer, &self.count, self.count);
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
    
}

CmdResult nmeaPollData(C::mip_interface& device, bool suppressAck, uint8_t count, const NMEAMessageFormat* formatEntries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, suppressAck);
    
    insert(serializer, count);
    
    assert(formatEntries);
    for(unsigned int i=0; i < count; i++)
        insert(serializer, formatEntries[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_NMEA_MESSAGE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const NmeaMessageFormat& self)
{
    insert(serializer, self.count);
    
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
    
}
void extract(Serializer& serializer, NmeaMessageFormat& self)
{
    C::extract_count(&serializer, &self.count, self.count);
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
    
}

CmdResult writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const NMEAMessageFormat* formatEntries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, count);
    
    assert(formatEntries);
    for(unsigned int i=0; i < count; i++)
        insert(serializer, formatEntries[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult readNmeaMessageFormat(C::mip_interface& device, uint8_t* countOut, uint8_t countOutMax, NMEAMessageFormat* formatEntriesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset, REPLY_NMEA_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, countOut, countOutMax);
        assert(formatEntriesOut);
        assert(countOut);
        for(unsigned int i=0; i < *countOut; i++)
            extract(deserializer, formatEntriesOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult loadNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}
CmdResult defaultNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const DeviceSettings& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, DeviceSettings& self)
{
    (void)serializer;
    (void)self;
}

CmdResult saveDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}
CmdResult loadDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}
CmdResult defaultDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, serializer.offset);
}
void insert(Serializer& serializer, const UartBaudrate& self)
{
    insert(serializer, self.baud);
    
}
void extract(Serializer& serializer, UartBaudrate& self)
{
    extract(serializer, self.baud);
    
}

CmdResult writeUartBaudrate(C::mip_interface& device, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, baud);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
}
CmdResult readUartBaudrate(C::mip_interface& device, uint32_t* baudOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset, REPLY_UART_BAUDRATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(baudOut);
        extract(deserializer, *baudOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
}
CmdResult loadUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
}
CmdResult defaultUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const FactoryStreaming& self)
{
    insert(serializer, self.action);
    
    insert(serializer, self.reserved);
    
}
void extract(Serializer& serializer, FactoryStreaming& self)
{
    extract(serializer, self.action);
    
    extract(serializer, self.reserved);
    
}

CmdResult factoryStreaming(C::mip_interface& device, FactoryStreaming::Action action, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, action);
    
    insert(serializer, reserved);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONFIGURE_FACTORY_STREAMING, buffer, serializer.offset);
}
void insert(Serializer& serializer, const DatastreamControl& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, DatastreamControl& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.enable);
    
}

CmdResult writeDatastreamControl(C::mip_interface& device, uint8_t descSet, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, descSet);
    
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
}
CmdResult readDatastreamControl(C::mip_interface& device, uint8_t descSet, bool* enabledOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset, REPLY_DATASTREAM_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, descSet);
        
        assert(enabledOut);
        extract(deserializer, *enabledOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
}
CmdResult loadDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
}
CmdResult defaultDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GnssSbasSettings& self)
{
    insert(serializer, self.enable_sbas);
    
    insert(serializer, self.sbas_options);
    
    insert(serializer, self.num_included_prns);
    
    for(unsigned int i=0; i < self.num_included_prns; i++)
        insert(serializer, self.included_prns[i]);
    
}
void extract(Serializer& serializer, GnssSbasSettings& self)
{
    extract(serializer, self.enable_sbas);
    
    extract(serializer, self.sbas_options);
    
    C::extract_count(&serializer, &self.num_included_prns, self.num_included_prns);
    for(unsigned int i=0; i < self.num_included_prns; i++)
        extract(serializer, self.included_prns[i]);
    
}

CmdResult writeGnssSbasSettings(C::mip_interface& device, uint8_t enableSbas, GnssSbasSettings::SBASOptions sbasOptions, uint8_t numIncludedPrns, const uint16_t* includedPrns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enableSbas);
    
    insert(serializer, sbasOptions);
    
    insert(serializer, numIncludedPrns);
    
    assert(includedPrns);
    for(unsigned int i=0; i < numIncludedPrns; i++)
        insert(serializer, includedPrns[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}
CmdResult readGnssSbasSettings(C::mip_interface& device, uint8_t* enableSbasOut, GnssSbasSettings::SBASOptions* sbasOptionsOut, uint8_t* numIncludedPrnsOut, uint8_t numIncludedPrnsOutMax, uint16_t* includedPrnsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset, REPLY_GNSS_SBAS_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableSbasOut);
        extract(deserializer, *enableSbasOut);
        
        assert(sbasOptionsOut);
        extract(deserializer, *sbasOptionsOut);
        
        C::extract_count(&deserializer, numIncludedPrnsOut, numIncludedPrnsOutMax);
        assert(includedPrnsOut);
        assert(numIncludedPrnsOut);
        for(unsigned int i=0; i < *numIncludedPrnsOut; i++)
            extract(deserializer, includedPrnsOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}
CmdResult loadGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}
CmdResult defaultGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GnssTimeAssistance& self)
{
    insert(serializer, self.tow);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.accuracy);
    
}
void extract(Serializer& serializer, GnssTimeAssistance& self)
{
    extract(serializer, self.tow);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.accuracy);
    
}

CmdResult writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t weekNumber, float accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, tow);
    
    insert(serializer, weekNumber);
    
    insert(serializer, accuracy);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, serializer.offset);
}
CmdResult readGnssTimeAssistance(C::mip_interface& device, double* towOut, uint16_t* weekNumberOut, float* accuracyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, serializer.offset, REPLY_GNSS_TIME_ASSISTANCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(towOut);
        extract(deserializer, *towOut);
        
        assert(weekNumberOut);
        extract(deserializer, *weekNumberOut);
        
        assert(accuracyOut);
        extract(deserializer, *accuracyOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const AdvLowpassFilter& self)
{
    insert(serializer, self.target_descriptor);
    
    insert(serializer, self.enable);
    
    insert(serializer, self.manual);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.reserved);
    
}
void extract(Serializer& serializer, AdvLowpassFilter& self)
{
    extract(serializer, self.target_descriptor);
    
    extract(serializer, self.enable);
    
    extract(serializer, self.manual);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.reserved);
    
}

CmdResult writeAdvLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, targetDescriptor);
    
    insert(serializer, enable);
    
    insert(serializer, manual);
    
    insert(serializer, frequency);
    
    insert(serializer, reserved);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}
CmdResult readAdvLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool* enableOut, bool* manualOut, uint16_t* frequencyOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset, REPLY_ADVANCED_DATA_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, targetDescriptor);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(manualOut);
        extract(deserializer, *manualOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(reservedOut);
        extract(deserializer, *reservedOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAdvLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}
CmdResult loadAdvLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}
CmdResult defaultAdvLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADVANCED_DATA_FILTER, buffer, serializer.offset);
}
void insert(Serializer& serializer, const PpsSource& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, PpsSource& self)
{
    extract(serializer, self.source);
    
}

CmdResult writePpsSource(C::mip_interface& device, PpsSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}
CmdResult readPpsSource(C::mip_interface& device, PpsSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset, REPLY_PPS_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult savePpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}
CmdResult loadPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}
CmdResult defaultPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GpioConfig& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.feature);
    
    insert(serializer, self.behavior);
    
    insert(serializer, self.pin_mode);
    
}
void extract(Serializer& serializer, GpioConfig& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.feature);
    
    extract(serializer, self.behavior);
    
    extract(serializer, self.pin_mode);
    
}

CmdResult writeGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature feature, GpioConfig::Behavior behavior, GpioConfig::PinMode pinMode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, pin);
    
    insert(serializer, feature);
    
    insert(serializer, behavior);
    
    insert(serializer, pinMode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
}
CmdResult readGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature* featureOut, GpioConfig::Behavior* behaviorOut, GpioConfig::PinMode* pinModeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset, REPLY_GPIO_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, pin);
        
        assert(featureOut);
        extract(deserializer, *featureOut);
        
        assert(behaviorOut);
        extract(deserializer, *behaviorOut);
        
        assert(pinModeOut);
        extract(deserializer, *pinModeOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
}
CmdResult loadGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
}
CmdResult defaultGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GpioState& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.state);
    
}
void extract(Serializer& serializer, GpioState& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.state);
    
}

CmdResult writeGpioState(C::mip_interface& device, uint8_t pin, bool state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, pin);
    
    insert(serializer, state);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, serializer.offset);
}
CmdResult readGpioState(C::mip_interface& device, uint8_t pin, bool* stateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, pin);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, serializer.offset, REPLY_GPIO_STATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, pin);
        
        assert(stateOut);
        extract(deserializer, *stateOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const Odometer& self)
{
    insert(serializer, self.mode);
    
    insert(serializer, self.scaling);
    
    insert(serializer, self.uncertainty);
    
}
void extract(Serializer& serializer, Odometer& self)
{
    extract(serializer, self.mode);
    
    extract(serializer, self.scaling);
    
    extract(serializer, self.uncertainty);
    
}

CmdResult writeOdometer(C::mip_interface& device, Odometer::Mode mode, float scaling, float uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    insert(serializer, scaling);
    
    insert(serializer, uncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}
CmdResult readOdometer(C::mip_interface& device, Odometer::Mode* modeOut, float* scalingOut, float* uncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset, REPLY_ODOMETER_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        assert(scalingOut);
        extract(deserializer, *scalingOut);
        
        assert(uncertaintyOut);
        extract(deserializer, *uncertaintyOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}
CmdResult loadOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}
CmdResult defaultOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GetEventSupport& self)
{
    insert(serializer, self.query);
    
}
void extract(Serializer& serializer, GetEventSupport& self)
{
    extract(serializer, self.query);
    
}

void insert(Serializer& serializer, const GetEventSupport::Info& self)
{
    insert(serializer, self.type);
    
    insert(serializer, self.count);
    
}
void extract(Serializer& serializer, GetEventSupport::Info& self)
{
    extract(serializer, self.type);
    
    extract(serializer, self.count);
    
}

CmdResult getEventSupport(C::mip_interface& device, GetEventSupport::Query query, uint8_t* maxInstancesOut, uint8_t* numEntriesOut, uint8_t numEntriesOutMax, GetEventSupport::Info* entriesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, query);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_SUPPORT, buffer, serializer.offset, REPLY_EVENT_SUPPORT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, query);
        
        assert(maxInstancesOut);
        extract(deserializer, *maxInstancesOut);
        
        C::extract_count(&deserializer, numEntriesOut, numEntriesOutMax);
        assert(entriesOut);
        assert(numEntriesOut);
        for(unsigned int i=0; i < *numEntriesOut; i++)
            extract(deserializer, entriesOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const EventControl& self)
{
    insert(serializer, self.instance);
    
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EventControl& self)
{
    extract(serializer, self.instance);
    
    extract(serializer, self.mode);
    
}

CmdResult writeEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, instance);
    
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
}
CmdResult readEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset, REPLY_EVENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, instance);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
}
CmdResult loadEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
}
CmdResult defaultEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GetEventTriggerStatus& self)
{
    insert(serializer, self.requested_count);
    
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
    
}
void extract(Serializer& serializer, GetEventTriggerStatus& self)
{
    C::extract_count(&serializer, &self.requested_count, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
    
}

void insert(Serializer& serializer, const GetEventTriggerStatus::Entry& self)
{
    insert(serializer, self.type);
    
    insert(serializer, self.status);
    
}
void extract(Serializer& serializer, GetEventTriggerStatus::Entry& self)
{
    extract(serializer, self.type);
    
    extract(serializer, self.status);
    
}

CmdResult getEventTriggerStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventTriggerStatus::Entry* triggersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, requestedCount);
    
    assert(requestedInstances);
    for(unsigned int i=0; i < requestedCount; i++)
        insert(serializer, requestedInstances[i]);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_STATUS, buffer, serializer.offset, REPLY_EVENT_TRIGGER_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, countOut, countOutMax);
        assert(triggersOut);
        assert(countOut);
        for(unsigned int i=0; i < *countOut; i++)
            extract(deserializer, triggersOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const GetEventActionStatus& self)
{
    insert(serializer, self.requested_count);
    
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
    
}
void extract(Serializer& serializer, GetEventActionStatus& self)
{
    C::extract_count(&serializer, &self.requested_count, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
    
}

void insert(Serializer& serializer, const GetEventActionStatus::Entry& self)
{
    insert(serializer, self.action_type);
    
    insert(serializer, self.trigger_id);
    
}
void extract(Serializer& serializer, GetEventActionStatus::Entry& self)
{
    extract(serializer, self.action_type);
    
    extract(serializer, self.trigger_id);
    
}

CmdResult getEventActionStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventActionStatus::Entry* actionsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, requestedCount);
    
    assert(requestedInstances);
    for(unsigned int i=0; i < requestedCount; i++)
        insert(serializer, requestedInstances[i]);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_STATUS, buffer, serializer.offset, REPLY_EVENT_ACTION_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        C::extract_count(&deserializer, countOut, countOutMax);
        assert(actionsOut);
        assert(countOut);
        for(unsigned int i=0; i < *countOut; i++)
            extract(deserializer, actionsOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const EventTrigger& self)
{
    insert(serializer, self.instance);
    
    insert(serializer, self.type);
    
    if( self.type == EventTrigger::Type::GPIO )
    {
        insert(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventTrigger::Type::THRESHOLD )
    {
        insert(serializer, self.parameters.threshold);
        
    }
    if( self.type == EventTrigger::Type::COMBINATION )
    {
        insert(serializer, self.parameters.combination);
        
    }
}
void extract(Serializer& serializer, EventTrigger& self)
{
    extract(serializer, self.instance);
    
    extract(serializer, self.type);
    
    if( self.type == EventTrigger::Type::GPIO )
    {
        extract(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventTrigger::Type::THRESHOLD )
    {
        extract(serializer, self.parameters.threshold);
        
    }
    if( self.type == EventTrigger::Type::COMBINATION )
    {
        extract(serializer, self.parameters.combination);
        
    }
}

void insert(Serializer& serializer, const EventTrigger::GpioParams& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EventTrigger::GpioParams& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.mode);
    
}

void insert(Serializer& serializer, const EventTrigger::ThresholdParams& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.field_desc);
    
    insert(serializer, self.param_id);
    
    insert(serializer, self.type);
    
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        insert(serializer, self.low_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        insert(serializer, self.int_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        insert(serializer, self.high_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        insert(serializer, self.interval);
        
    }
}
void extract(Serializer& serializer, EventTrigger::ThresholdParams& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.field_desc);
    
    extract(serializer, self.param_id);
    
    extract(serializer, self.type);
    
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        extract(serializer, self.low_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        extract(serializer, self.int_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        extract(serializer, self.high_thres);
        
    }
    if( self.type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        extract(serializer, self.interval);
        
    }
}

void insert(Serializer& serializer, const EventTrigger::CombinationParams& self)
{
    insert(serializer, self.logic_table);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.input_triggers[i]);
    
}
void extract(Serializer& serializer, EventTrigger::CombinationParams& self)
{
    extract(serializer, self.logic_table);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.input_triggers[i]);
    
}

CmdResult writeEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type type, const EventTrigger::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, instance);
    
    insert(serializer, type);
    
    if( type == EventTrigger::Type::GPIO )
    {
        insert(serializer, parameters.gpio);
        
    }
    if( type == EventTrigger::Type::THRESHOLD )
    {
        insert(serializer, parameters.threshold);
        
    }
    if( type == EventTrigger::Type::COMBINATION )
    {
        insert(serializer, parameters.combination);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}
CmdResult readEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type* typeOut, EventTrigger::Parameters* parametersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset, REPLY_EVENT_TRIGGER_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, instance);
        
        assert(typeOut);
        extract(deserializer, *typeOut);
        
        if( *typeOut == EventTrigger::Type::GPIO )
        {
            extract(deserializer, parametersOut->gpio);
            
        }
        if( *typeOut == EventTrigger::Type::THRESHOLD )
        {
            extract(deserializer, parametersOut->threshold);
            
        }
        if( *typeOut == EventTrigger::Type::COMBINATION )
        {
            extract(deserializer, parametersOut->combination);
            
        }
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}
CmdResult loadEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}
CmdResult defaultEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, serializer.offset);
}
void insert(Serializer& serializer, const EventAction& self)
{
    insert(serializer, self.instance);
    
    insert(serializer, self.trigger);
    
    insert(serializer, self.type);
    
    if( self.type == EventAction::Type::GPIO )
    {
        insert(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventAction::Type::MESSAGE )
    {
        insert(serializer, self.parameters.message);
        
    }
}
void extract(Serializer& serializer, EventAction& self)
{
    extract(serializer, self.instance);
    
    extract(serializer, self.trigger);
    
    extract(serializer, self.type);
    
    if( self.type == EventAction::Type::GPIO )
    {
        extract(serializer, self.parameters.gpio);
        
    }
    if( self.type == EventAction::Type::MESSAGE )
    {
        extract(serializer, self.parameters.message);
        
    }
}

void insert(Serializer& serializer, const EventAction::GpioParams& self)
{
    insert(serializer, self.pin);
    
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EventAction::GpioParams& self)
{
    extract(serializer, self.pin);
    
    extract(serializer, self.mode);
    
}

void insert(Serializer& serializer, const EventAction::MessageParams& self)
{
    insert(serializer, self.desc_set);
    
    insert(serializer, self.decimation);
    
    insert(serializer, self.num_fields);
    
    for(unsigned int i=0; i < self.num_fields; i++)
        insert(serializer, self.descriptors[i]);
    
}
void extract(Serializer& serializer, EventAction::MessageParams& self)
{
    extract(serializer, self.desc_set);
    
    extract(serializer, self.decimation);
    
    C::extract_count(&serializer, &self.num_fields, self.num_fields);
    for(unsigned int i=0; i < self.num_fields; i++)
        extract(serializer, self.descriptors[i]);
    
}

CmdResult writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, EventAction::Type type, const EventAction::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, instance);
    
    insert(serializer, trigger);
    
    insert(serializer, type);
    
    if( type == EventAction::Type::GPIO )
    {
        insert(serializer, parameters.gpio);
        
    }
    if( type == EventAction::Type::MESSAGE )
    {
        insert(serializer, parameters.message);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}
CmdResult readEventAction(C::mip_interface& device, uint8_t instance, uint8_t* triggerOut, EventAction::Type* typeOut, EventAction::Parameters* parametersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset, REPLY_EVENT_ACTION_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, instance);
        
        assert(triggerOut);
        extract(deserializer, *triggerOut);
        
        assert(typeOut);
        extract(deserializer, *typeOut);
        
        if( *typeOut == EventAction::Type::GPIO )
        {
            extract(deserializer, parametersOut->gpio);
            
        }
        if( *typeOut == EventAction::Type::MESSAGE )
        {
            extract(deserializer, parametersOut->message);
            
        }
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}
CmdResult loadEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}
CmdResult defaultEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, serializer.offset);
}
void insert(Serializer& serializer, const AccelBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    
}
void extract(Serializer& serializer, AccelBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    
}

CmdResult writeAccelBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(bias);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, bias[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}
CmdResult readAccelBias(C::mip_interface& device, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset, REPLY_ACCEL_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, biasOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}
CmdResult loadAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}
CmdResult defaultAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, serializer.offset);
}
void insert(Serializer& serializer, const GyroBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    
}
void extract(Serializer& serializer, GyroBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    
}

CmdResult writeGyroBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(bias);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, bias[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}
CmdResult readGyroBias(C::mip_interface& device, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset, REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, biasOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}
CmdResult loadGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}
CmdResult defaultGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, serializer.offset);
}
void insert(Serializer& serializer, const CaptureGyroBias& self)
{
    insert(serializer, self.averaging_time_ms);
    
}
void extract(Serializer& serializer, CaptureGyroBias& self)
{
    extract(serializer, self.averaging_time_ms);
    
}

CmdResult captureGyroBias(C::mip_interface& device, uint16_t averagingTimeMs, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, averagingTimeMs);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CAPTURE_GYRO_BIAS, buffer, serializer.offset, REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, biasOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const MagHardIronOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, MagHardIronOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

CmdResult writeMagHardIronOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
}
CmdResult readMagHardIronOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset, REPLY_HARD_IRON_OFFSET_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
}
CmdResult loadMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
}
CmdResult defaultMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, serializer.offset);
}
void insert(Serializer& serializer, const MagSoftIronMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, MagSoftIronMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.offset[i]);
    
}

CmdResult writeMagSoftIronMatrix(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
}
CmdResult readMagSoftIronMatrix(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset, REPLY_SOFT_IRON_COMP_MATRIX, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut);
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, offsetOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
}
CmdResult loadMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
}
CmdResult defaultMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, serializer.offset);
}
void insert(Serializer& serializer, const Sensor2VehicleTransformEuler& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(Serializer& serializer, Sensor2VehicleTransformEuler& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.yaw);
    
}

CmdResult writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, roll);
    
    insert(serializer, pitch);
    
    insert(serializer, yaw);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}
CmdResult readSensor2VehicleTransformEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rollOut);
        extract(deserializer, *rollOut);
        
        assert(pitchOut);
        extract(deserializer, *pitchOut);
        
        assert(yawOut);
        extract(deserializer, *yawOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}
CmdResult loadSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}
CmdResult defaultSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, serializer.offset);
}
void insert(Serializer& serializer, const Sensor2VehicleTransformQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
    
}
void extract(Serializer& serializer, Sensor2VehicleTransformQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
    
}

CmdResult writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(q);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, q[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}
CmdResult readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* qOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(qOut);
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, qOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}
CmdResult loadSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}
CmdResult defaultSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, serializer.offset);
}
void insert(Serializer& serializer, const Sensor2VehicleTransformDcm& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
    
}
void extract(Serializer& serializer, Sensor2VehicleTransformDcm& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
    
}

CmdResult writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(dcm);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}
CmdResult readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset, REPLY_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(dcmOut);
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, dcmOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}
CmdResult loadSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}
CmdResult defaultSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ComplementaryFilter& self)
{
    insert(serializer, self.pitch_roll_enable);
    
    insert(serializer, self.heading_enable);
    
    insert(serializer, self.pitch_roll_time_constant);
    
    insert(serializer, self.heading_time_constant);
    
}
void extract(Serializer& serializer, ComplementaryFilter& self)
{
    extract(serializer, self.pitch_roll_enable);
    
    extract(serializer, self.heading_enable);
    
    extract(serializer, self.pitch_roll_time_constant);
    
    extract(serializer, self.heading_time_constant);
    
}

CmdResult writeComplementaryFilter(C::mip_interface& device, bool pitchRollEnable, bool headingEnable, float pitchRollTimeConstant, float headingTimeConstant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, pitchRollEnable);
    
    insert(serializer, headingEnable);
    
    insert(serializer, pitchRollTimeConstant);
    
    insert(serializer, headingTimeConstant);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}
CmdResult readComplementaryFilter(C::mip_interface& device, bool* pitchRollEnableOut, bool* headingEnableOut, float* pitchRollTimeConstantOut, float* headingTimeConstantOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset, REPLY_LEGACY_COMP_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(pitchRollEnableOut);
        extract(deserializer, *pitchRollEnableOut);
        
        assert(headingEnableOut);
        extract(deserializer, *headingEnableOut);
        
        assert(pitchRollTimeConstantOut);
        extract(deserializer, *pitchRollTimeConstantOut);
        
        assert(headingTimeConstantOut);
        extract(deserializer, *headingTimeConstantOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}
CmdResult loadComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}
CmdResult defaultComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, serializer.offset);
}
void insert(Serializer& serializer, const SensorRange& self)
{
    insert(serializer, self.sensor);
    
    insert(serializer, self.setting);
    
}
void extract(Serializer& serializer, SensorRange& self)
{
    extract(serializer, self.sensor);
    
    extract(serializer, self.setting);
    
}

CmdResult writeSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, sensor);
    
    insert(serializer, setting);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
}
CmdResult readSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t* settingOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset, REPLY_SENSOR_RANGE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, sensor);
        
        assert(settingOut);
        extract(deserializer, *settingOut);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
}
CmdResult loadSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
}
CmdResult defaultSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, serializer.offset);
}
void insert(Serializer& serializer, const CalibratedSensorRanges& self)
{
    insert(serializer, self.sensor);
    
}
void extract(Serializer& serializer, CalibratedSensorRanges& self)
{
    extract(serializer, self.sensor);
    
}

void insert(Serializer& serializer, const CalibratedSensorRanges::Entry& self)
{
    insert(serializer, self.setting);
    
    insert(serializer, self.range);
    
}
void extract(Serializer& serializer, CalibratedSensorRanges::Entry& self)
{
    extract(serializer, self.setting);
    
    extract(serializer, self.range);
    
}

CmdResult calibratedSensorRanges(C::mip_interface& device, SensorRangeType sensor, uint8_t* numRangesOut, uint8_t numRangesOutMax, CalibratedSensorRanges::Entry* rangesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, sensor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CALIBRATED_RANGES, buffer, serializer.offset, REPLY_CALIBRATED_RANGES, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, sensor);
        
        C::extract_count(&deserializer, numRangesOut, numRangesOutMax);
        assert(rangesOut);
        assert(numRangesOut);
        for(unsigned int i=0; i < *numRangesOut; i++)
            extract(deserializer, rangesOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}

} // namespace commands_3dm
} // namespace mip

