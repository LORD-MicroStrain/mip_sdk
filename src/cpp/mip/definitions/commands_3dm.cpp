
#include "commands_3dm.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_3dm {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void NmeaMessage::insert(Serializer& serializer) const
{
    serializer.insert(message_id);
    
    serializer.insert(talker_id);
    
    serializer.insert(source_desc_set);
    
    serializer.insert(decimation);
    
}
void NmeaMessage::extract(Serializer& serializer)
{
    serializer.extract(message_id);
    
    serializer.extract(talker_id);
    
    serializer.extract(source_desc_set);
    
    serializer.extract(decimation);
    
}

void PollImuMessage::insert(Serializer& serializer) const
{
    serializer.insert(suppress_ack);
    
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void PollImuMessage::extract(Serializer& serializer)
{
    serializer.extract(suppress_ack);
    
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<PollImuMessage> pollImuMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(suppressAck);
    
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_IMU_MESSAGE, buffer, (uint8_t)serializer.usedLength());
}
void PollGnssMessage::insert(Serializer& serializer) const
{
    serializer.insert(suppress_ack);
    
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void PollGnssMessage::extract(Serializer& serializer)
{
    serializer.extract(suppress_ack);
    
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<PollGnssMessage> pollGnssMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(suppressAck);
    
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_GNSS_MESSAGE, buffer, (uint8_t)serializer.usedLength());
}
void PollFilterMessage::insert(Serializer& serializer) const
{
    serializer.insert(suppress_ack);
    
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void PollFilterMessage::extract(Serializer& serializer)
{
    serializer.extract(suppress_ack);
    
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<PollFilterMessage> pollFilterMessage(C::mip_interface& device, bool suppressAck, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(suppressAck);
    
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_FILTER_MESSAGE, buffer, (uint8_t)serializer.usedLength());
}
void NmeaPollData::insert(Serializer& serializer) const
{
    serializer.insert(suppress_ack);
    
    serializer.insert(count);
    
    for(unsigned int i=0; i < count; i++)
        serializer.insert(format_entries[i]);
    
}
void NmeaPollData::extract(Serializer& serializer)
{
    serializer.extract(suppress_ack);
    
    serializer.extract_count(count, sizeof(format_entries)/sizeof(format_entries[0]));
    for(unsigned int i=0; i < count; i++)
        serializer.extract(format_entries[i]);
    
}

TypedResult<NmeaPollData> nmeaPollData(C::mip_interface& device, bool suppressAck, uint8_t count, const NmeaMessage* formatEntries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(suppressAck);
    
    serializer.insert(count);
    
    assert(formatEntries || (count == 0));
    for(unsigned int i=0; i < count; i++)
        serializer.insert(formatEntries[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_NMEA_MESSAGE, buffer, (uint8_t)serializer.usedLength());
}
void ImuGetBaseRate::insert(Serializer& serializer) const
{
    (void)serializer;
}
void ImuGetBaseRate::extract(Serializer& serializer)
{
    (void)serializer;
}

void ImuGetBaseRate::Response::insert(Serializer& serializer) const
{
    serializer.insert(rate);
    
}
void ImuGetBaseRate::Response::extract(Serializer& serializer)
{
    serializer.extract(rate);
    
}

TypedResult<ImuGetBaseRate> imuGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<ImuGetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_IMU_BASE_RATE, NULL, 0, REPLY_IMU_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        deserializer.extract(*rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GnssGetBaseRate::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GnssGetBaseRate::extract(Serializer& serializer)
{
    (void)serializer;
}

void GnssGetBaseRate::Response::insert(Serializer& serializer) const
{
    serializer.insert(rate);
    
}
void GnssGetBaseRate::Response::extract(Serializer& serializer)
{
    serializer.extract(rate);
    
}

TypedResult<GnssGetBaseRate> gnssGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GnssGetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_GNSS_BASE_RATE, NULL, 0, REPLY_GNSS_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        deserializer.extract(*rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void ImuMessageFormat::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(num_descriptors);
        
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.insert(descriptors[i]);
        
    }
}
void ImuMessageFormat::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.extract(descriptors[i]);
        
    }
}

void ImuMessageFormat::Response::insert(Serializer& serializer) const
{
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void ImuMessageFormat::Response::extract(Serializer& serializer)
{
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<ImuMessageFormat> writeImuMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ImuMessageFormat> readImuMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ImuMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength(), REPLY_IMU_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            deserializer.extract(descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ImuMessageFormat> saveImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ImuMessageFormat> loadImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ImuMessageFormat> defaultImuMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
void GnssMessageFormat::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(num_descriptors);
        
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.insert(descriptors[i]);
        
    }
}
void GnssMessageFormat::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.extract(descriptors[i]);
        
    }
}

void GnssMessageFormat::Response::insert(Serializer& serializer) const
{
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void GnssMessageFormat::Response::extract(Serializer& serializer)
{
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<GnssMessageFormat> writeGnssMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssMessageFormat> readGnssMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength(), REPLY_GNSS_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            deserializer.extract(descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssMessageFormat> saveGnssMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssMessageFormat> loadGnssMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssMessageFormat> defaultGnssMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
void FilterMessageFormat::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(num_descriptors);
        
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.insert(descriptors[i]);
        
    }
}
void FilterMessageFormat::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.extract(descriptors[i]);
        
    }
}

void FilterMessageFormat::Response::insert(Serializer& serializer) const
{
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void FilterMessageFormat::Response::extract(Serializer& serializer)
{
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<FilterMessageFormat> writeFilterMessageFormat(C::mip_interface& device, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<FilterMessageFormat> readFilterMessageFormat(C::mip_interface& device, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<FilterMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength(), REPLY_FILTER_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            deserializer.extract(descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<FilterMessageFormat> saveFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<FilterMessageFormat> loadFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<FilterMessageFormat> defaultFilterMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FILTER_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
void FilterGetBaseRate::insert(Serializer& serializer) const
{
    (void)serializer;
}
void FilterGetBaseRate::extract(Serializer& serializer)
{
    (void)serializer;
}

void FilterGetBaseRate::Response::insert(Serializer& serializer) const
{
    serializer.insert(rate);
    
}
void FilterGetBaseRate::Response::extract(Serializer& serializer)
{
    serializer.extract(rate);
    
}

TypedResult<FilterGetBaseRate> filterGetBaseRate(C::mip_interface& device, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<FilterGetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_FILTER_BASE_RATE, NULL, 0, REPLY_FILTER_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rateOut);
        deserializer.extract(*rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void NmeaMessageFormat::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(count);
        
        for(unsigned int i=0; i < count; i++)
            serializer.insert(format_entries[i]);
        
    }
}
void NmeaMessageFormat::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract_count(count, sizeof(format_entries)/sizeof(format_entries[0]));
        for(unsigned int i=0; i < count; i++)
            serializer.extract(format_entries[i]);
        
    }
}

void NmeaMessageFormat::Response::insert(Serializer& serializer) const
{
    serializer.insert(count);
    
    for(unsigned int i=0; i < count; i++)
        serializer.insert(format_entries[i]);
    
}
void NmeaMessageFormat::Response::extract(Serializer& serializer)
{
    serializer.extract_count(count, sizeof(format_entries)/sizeof(format_entries[0]));
    for(unsigned int i=0; i < count; i++)
        serializer.extract(format_entries[i]);
    
}

TypedResult<NmeaMessageFormat> writeNmeaMessageFormat(C::mip_interface& device, uint8_t count, const NmeaMessage* formatEntries)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(count);
    
    assert(formatEntries || (count == 0));
    for(unsigned int i=0; i < count; i++)
        serializer.insert(formatEntries[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<NmeaMessageFormat> readNmeaMessageFormat(C::mip_interface& device, uint8_t* countOut, uint8_t countOutMax, NmeaMessage* formatEntriesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<NmeaMessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength(), REPLY_NMEA_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*countOut, countOutMax);
        assert(formatEntriesOut || (countOut == 0));
        for(unsigned int i=0; i < *countOut; i++)
            deserializer.extract(formatEntriesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<NmeaMessageFormat> saveNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<NmeaMessageFormat> loadNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<NmeaMessageFormat> defaultNmeaMessageFormat(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_NMEA_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
void PollData::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(suppress_ack);
    
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void PollData::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract(suppress_ack);
    
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<PollData> pollData(C::mip_interface& device, uint8_t descSet, bool suppressAck, uint8_t numDescriptors, const uint8_t* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(descSet);
    
    serializer.insert(suppressAck);
    
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POLL_DATA, buffer, (uint8_t)serializer.usedLength());
}
void GetBaseRate::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
}
void GetBaseRate::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
}

void GetBaseRate::Response::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(rate);
    
}
void GetBaseRate::Response::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract(rate);
    
}

TypedResult<GetBaseRate> getBaseRate(C::mip_interface& device, uint8_t descSet, uint16_t* rateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetBaseRate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_BASE_RATE, buffer, (uint8_t)serializer.usedLength(), REPLY_BASE_RATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(descSet);
        
        assert(rateOut);
        deserializer.extract(*rateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void MessageFormat::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(desc_set);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(num_descriptors);
        
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.insert(descriptors[i]);
        
    }
}
void MessageFormat::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(desc_set);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
        for(unsigned int i=0; i < num_descriptors; i++)
            serializer.extract(descriptors[i]);
        
    }
}

void MessageFormat::Response::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(num_descriptors);
    
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.insert(descriptors[i]);
    
}
void MessageFormat::Response::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract_count(num_descriptors, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_descriptors; i++)
        serializer.extract(descriptors[i]);
    
}

TypedResult<MessageFormat> writeMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t numDescriptors, const DescriptorRate* descriptors)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(descSet);
    
    serializer.insert(numDescriptors);
    
    assert(descriptors || (numDescriptors == 0));
    for(unsigned int i=0; i < numDescriptors; i++)
        serializer.insert(descriptors[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MessageFormat> readMessageFormat(C::mip_interface& device, uint8_t descSet, uint8_t* numDescriptorsOut, uint8_t numDescriptorsOutMax, DescriptorRate* descriptorsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MessageFormat> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength(), REPLY_MESSAGE_FORMAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(descSet);
        
        deserializer.extract_count(*numDescriptorsOut, numDescriptorsOutMax);
        assert(descriptorsOut || (numDescriptorsOut == 0));
        for(unsigned int i=0; i < *numDescriptorsOut; i++)
            deserializer.extract(descriptorsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MessageFormat> saveMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MessageFormat> loadMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MessageFormat> defaultMessageFormat(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MESSAGE_FORMAT, buffer, (uint8_t)serializer.usedLength());
}
void FactoryStreaming::insert(Serializer& serializer) const
{
    serializer.insert(action);
    
    serializer.insert(reserved);
    
}
void FactoryStreaming::extract(Serializer& serializer)
{
    serializer.extract(action);
    
    serializer.extract(reserved);
    
}

TypedResult<FactoryStreaming> factoryStreaming(C::mip_interface& device, FactoryStreaming::Action action, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(action);
    
    serializer.insert(reserved);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONFIGURE_FACTORY_STREAMING, buffer, (uint8_t)serializer.usedLength());
}
void DatastreamControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(desc_set);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void DatastreamControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(desc_set);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void DatastreamControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(enabled);
    
}
void DatastreamControl::Response::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract(enabled);
    
}

TypedResult<DatastreamControl> writeDatastreamControl(C::mip_interface& device, uint8_t descSet, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(descSet);
    
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<DatastreamControl> readDatastreamControl(C::mip_interface& device, uint8_t descSet, bool* enabledOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<DatastreamControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)serializer.usedLength(), REPLY_DATASTREAM_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(descSet);
        
        assert(enabledOut);
        deserializer.extract(*enabledOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<DatastreamControl> saveDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<DatastreamControl> loadDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<DatastreamControl> defaultDatastreamControl(C::mip_interface& device, uint8_t descSet)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(descSet);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONTROL_DATA_STREAM, buffer, (uint8_t)serializer.usedLength());
}
void ConstellationSettings::Settings::insert(Serializer& serializer) const
{
    serializer.insert(constellation_id);
    
    serializer.insert(enable);
    
    serializer.insert(reserved_channels);
    
    serializer.insert(max_channels);
    
    serializer.insert(option_flags);
    
}
void ConstellationSettings::Settings::extract(Serializer& serializer)
{
    serializer.extract(constellation_id);
    
    serializer.extract(enable);
    
    serializer.extract(reserved_channels);
    
    serializer.extract(max_channels);
    
    serializer.extract(option_flags);
    
}

void ConstellationSettings::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(max_channels);
        
        serializer.insert(config_count);
        
        for(unsigned int i=0; i < config_count; i++)
            serializer.insert(settings[i]);
        
    }
}
void ConstellationSettings::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(max_channels);
        
        serializer.extract_count(config_count, sizeof(settings)/sizeof(settings[0]));
        for(unsigned int i=0; i < config_count; i++)
            serializer.extract(settings[i]);
        
    }
}

void ConstellationSettings::Response::insert(Serializer& serializer) const
{
    serializer.insert(max_channels_available);
    
    serializer.insert(max_channels_use);
    
    serializer.insert(config_count);
    
    for(unsigned int i=0; i < config_count; i++)
        serializer.insert(settings[i]);
    
}
void ConstellationSettings::Response::extract(Serializer& serializer)
{
    serializer.extract(max_channels_available);
    
    serializer.extract(max_channels_use);
    
    serializer.extract_count(config_count, sizeof(settings)/sizeof(settings[0]));
    for(unsigned int i=0; i < config_count; i++)
        serializer.extract(settings[i]);
    
}

TypedResult<ConstellationSettings> writeConstellationSettings(C::mip_interface& device, uint16_t maxChannels, uint8_t configCount, const ConstellationSettings::Settings* settings)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(maxChannels);
    
    serializer.insert(configCount);
    
    assert(settings || (configCount == 0));
    for(unsigned int i=0; i < configCount; i++)
        serializer.insert(settings[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConstellationSettings> readConstellationSettings(C::mip_interface& device, uint16_t* maxChannelsAvailableOut, uint16_t* maxChannelsUseOut, uint8_t* configCountOut, uint8_t configCountOutMax, ConstellationSettings::Settings* settingsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ConstellationSettings> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)serializer.usedLength(), REPLY_GNSS_CONSTELLATION_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(maxChannelsAvailableOut);
        deserializer.extract(*maxChannelsAvailableOut);
        
        assert(maxChannelsUseOut);
        deserializer.extract(*maxChannelsUseOut);
        
        deserializer.extract_count(*configCountOut, configCountOutMax);
        assert(settingsOut || (configCountOut == 0));
        for(unsigned int i=0; i < *configCountOut; i++)
            deserializer.extract(settingsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ConstellationSettings> saveConstellationSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConstellationSettings> loadConstellationSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConstellationSettings> defaultConstellationSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_CONSTELLATION_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
void GnssSbasSettings::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable_sbas);
        
        serializer.insert(sbas_options);
        
        serializer.insert(num_included_prns);
        
        for(unsigned int i=0; i < num_included_prns; i++)
            serializer.insert(included_prns[i]);
        
    }
}
void GnssSbasSettings::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable_sbas);
        
        serializer.extract(sbas_options);
        
        serializer.extract_count(num_included_prns, sizeof(included_prns)/sizeof(included_prns[0]));
        for(unsigned int i=0; i < num_included_prns; i++)
            serializer.extract(included_prns[i]);
        
    }
}

void GnssSbasSettings::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable_sbas);
    
    serializer.insert(sbas_options);
    
    serializer.insert(num_included_prns);
    
    for(unsigned int i=0; i < num_included_prns; i++)
        serializer.insert(included_prns[i]);
    
}
void GnssSbasSettings::Response::extract(Serializer& serializer)
{
    serializer.extract(enable_sbas);
    
    serializer.extract(sbas_options);
    
    serializer.extract_count(num_included_prns, sizeof(included_prns)/sizeof(included_prns[0]));
    for(unsigned int i=0; i < num_included_prns; i++)
        serializer.extract(included_prns[i]);
    
}

TypedResult<GnssSbasSettings> writeGnssSbasSettings(C::mip_interface& device, uint8_t enableSbas, GnssSbasSettings::SBASOptions sbasOptions, uint8_t numIncludedPrns, const uint16_t* includedPrns)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enableSbas);
    
    serializer.insert(sbasOptions);
    
    serializer.insert(numIncludedPrns);
    
    assert(includedPrns || (numIncludedPrns == 0));
    for(unsigned int i=0; i < numIncludedPrns; i++)
        serializer.insert(includedPrns[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssSbasSettings> readGnssSbasSettings(C::mip_interface& device, uint8_t* enableSbasOut, GnssSbasSettings::SBASOptions* sbasOptionsOut, uint8_t* numIncludedPrnsOut, uint8_t numIncludedPrnsOutMax, uint16_t* includedPrnsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssSbasSettings> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)serializer.usedLength(), REPLY_GNSS_SBAS_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableSbasOut);
        deserializer.extract(*enableSbasOut);
        
        assert(sbasOptionsOut);
        deserializer.extract(*sbasOptionsOut);
        
        deserializer.extract_count(*numIncludedPrnsOut, numIncludedPrnsOutMax);
        assert(includedPrnsOut || (numIncludedPrnsOut == 0));
        for(unsigned int i=0; i < *numIncludedPrnsOut; i++)
            deserializer.extract(includedPrnsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssSbasSettings> saveGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssSbasSettings> loadGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssSbasSettings> defaultGnssSbasSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SBAS_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
void GnssAssistedFix::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(option);
        
        serializer.insert(flags);
        
    }
}
void GnssAssistedFix::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(option);
        
        serializer.extract(flags);
        
    }
}

void GnssAssistedFix::Response::insert(Serializer& serializer) const
{
    serializer.insert(option);
    
    serializer.insert(flags);
    
}
void GnssAssistedFix::Response::extract(Serializer& serializer)
{
    serializer.extract(option);
    
    serializer.extract(flags);
    
}

TypedResult<GnssAssistedFix> writeGnssAssistedFix(C::mip_interface& device, GnssAssistedFix::AssistedFixOption option, uint8_t flags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(option);
    
    serializer.insert(flags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssAssistedFix> readGnssAssistedFix(C::mip_interface& device, GnssAssistedFix::AssistedFixOption* optionOut, uint8_t* flagsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssAssistedFix> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)serializer.usedLength(), REPLY_GNSS_ASSISTED_FIX_SETTINGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(optionOut);
        deserializer.extract(*optionOut);
        
        assert(flagsOut);
        deserializer.extract(*flagsOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssAssistedFix> saveGnssAssistedFix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssAssistedFix> loadGnssAssistedFix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssAssistedFix> defaultGnssAssistedFix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_ASSISTED_FIX_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
void GnssTimeAssistance::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(tow);
        
        serializer.insert(week_number);
        
        serializer.insert(accuracy);
        
    }
}
void GnssTimeAssistance::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(tow);
        
        serializer.extract(week_number);
        
        serializer.extract(accuracy);
        
    }
}

void GnssTimeAssistance::Response::insert(Serializer& serializer) const
{
    serializer.insert(tow);
    
    serializer.insert(week_number);
    
    serializer.insert(accuracy);
    
}
void GnssTimeAssistance::Response::extract(Serializer& serializer)
{
    serializer.extract(tow);
    
    serializer.extract(week_number);
    
    serializer.extract(accuracy);
    
}

TypedResult<GnssTimeAssistance> writeGnssTimeAssistance(C::mip_interface& device, double tow, uint16_t weekNumber, float accuracy)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(tow);
    
    serializer.insert(weekNumber);
    
    serializer.insert(accuracy);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GnssTimeAssistance> readGnssTimeAssistance(C::mip_interface& device, double* towOut, uint16_t* weekNumberOut, float* accuracyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssTimeAssistance> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_TIME_ASSISTANCE, buffer, (uint8_t)serializer.usedLength(), REPLY_GNSS_TIME_ASSISTANCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(towOut);
        deserializer.extract(*towOut);
        
        assert(weekNumberOut);
        deserializer.extract(*weekNumberOut);
        
        assert(accuracyOut);
        deserializer.extract(*accuracyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void PpsSource::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(source);
        
    }
}
void PpsSource::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(source);
        
    }
}

void PpsSource::Response::insert(Serializer& serializer) const
{
    serializer.insert(source);
    
}
void PpsSource::Response::extract(Serializer& serializer)
{
    serializer.extract(source);
    
}

TypedResult<PpsSource> writePpsSource(C::mip_interface& device, PpsSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PpsSource> readPpsSource(C::mip_interface& device, PpsSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<PpsSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)serializer.usedLength(), REPLY_PPS_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        deserializer.extract(*sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<PpsSource> savePpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PpsSource> loadPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<PpsSource> defaultPpsSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PPS_SOURCE, buffer, (uint8_t)serializer.usedLength());
}
void GetEventSupport::Info::insert(Serializer& serializer) const
{
    serializer.insert(type);
    
    serializer.insert(count);
    
}
void GetEventSupport::Info::extract(Serializer& serializer)
{
    serializer.extract(type);
    
    serializer.extract(count);
    
}

void GetEventSupport::insert(Serializer& serializer) const
{
    serializer.insert(query);
    
}
void GetEventSupport::extract(Serializer& serializer)
{
    serializer.extract(query);
    
}

void GetEventSupport::Response::insert(Serializer& serializer) const
{
    serializer.insert(query);
    
    serializer.insert(max_instances);
    
    serializer.insert(num_entries);
    
    for(unsigned int i=0; i < num_entries; i++)
        serializer.insert(entries[i]);
    
}
void GetEventSupport::Response::extract(Serializer& serializer)
{
    serializer.extract(query);
    
    serializer.extract(max_instances);
    
    serializer.extract_count(num_entries, sizeof(entries)/sizeof(entries[0]));
    for(unsigned int i=0; i < num_entries; i++)
        serializer.extract(entries[i]);
    
}

TypedResult<GetEventSupport> getEventSupport(C::mip_interface& device, GetEventSupport::Query query, uint8_t* maxInstancesOut, uint8_t* numEntriesOut, uint8_t numEntriesOutMax, GetEventSupport::Info* entriesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(query);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetEventSupport> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_SUPPORT, buffer, (uint8_t)serializer.usedLength(), REPLY_EVENT_SUPPORT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(query);
        
        assert(maxInstancesOut);
        deserializer.extract(*maxInstancesOut);
        
        deserializer.extract_count(*numEntriesOut, numEntriesOutMax);
        assert(entriesOut || (numEntriesOut == 0));
        for(unsigned int i=0; i < *numEntriesOut; i++)
            deserializer.extract(entriesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void EventControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(instance);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(mode);
        
    }
}
void EventControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(instance);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(mode);
        
    }
}

void EventControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(instance);
    
    serializer.insert(mode);
    
}
void EventControl::Response::extract(Serializer& serializer)
{
    serializer.extract(instance);
    
    serializer.extract(mode);
    
}

TypedResult<EventControl> writeEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(instance);
    
    serializer.insert(mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventControl> readEventControl(C::mip_interface& device, uint8_t instance, EventControl::Mode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EventControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_EVENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(instance);
        
        assert(modeOut);
        deserializer.extract(*modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EventControl> saveEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventControl> loadEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventControl> defaultEventControl(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
void GetEventTriggerStatus::Entry::insert(Serializer& serializer) const
{
    serializer.insert(type);
    
    serializer.insert(status);
    
}
void GetEventTriggerStatus::Entry::extract(Serializer& serializer)
{
    serializer.extract(type);
    
    serializer.extract(status);
    
}

void GetEventTriggerStatus::insert(Serializer& serializer) const
{
    serializer.insert(requested_count);
    
    for(unsigned int i=0; i < requested_count; i++)
        serializer.insert(requested_instances[i]);
    
}
void GetEventTriggerStatus::extract(Serializer& serializer)
{
    serializer.extract_count(requested_count, sizeof(requested_instances)/sizeof(requested_instances[0]));
    for(unsigned int i=0; i < requested_count; i++)
        serializer.extract(requested_instances[i]);
    
}

void GetEventTriggerStatus::Response::insert(Serializer& serializer) const
{
    serializer.insert(count);
    
    for(unsigned int i=0; i < count; i++)
        serializer.insert(triggers[i]);
    
}
void GetEventTriggerStatus::Response::extract(Serializer& serializer)
{
    serializer.extract_count(count, sizeof(triggers)/sizeof(triggers[0]));
    for(unsigned int i=0; i < count; i++)
        serializer.extract(triggers[i]);
    
}

TypedResult<GetEventTriggerStatus> getEventTriggerStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventTriggerStatus::Entry* triggersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(requestedCount);
    
    assert(requestedInstances || (requestedCount == 0));
    for(unsigned int i=0; i < requestedCount; i++)
        serializer.insert(requestedInstances[i]);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetEventTriggerStatus> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_STATUS, buffer, (uint8_t)serializer.usedLength(), REPLY_EVENT_TRIGGER_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*countOut, countOutMax);
        assert(triggersOut || (countOut == 0));
        for(unsigned int i=0; i < *countOut; i++)
            deserializer.extract(triggersOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetEventActionStatus::Entry::insert(Serializer& serializer) const
{
    serializer.insert(action_type);
    
    serializer.insert(trigger_id);
    
}
void GetEventActionStatus::Entry::extract(Serializer& serializer)
{
    serializer.extract(action_type);
    
    serializer.extract(trigger_id);
    
}

void GetEventActionStatus::insert(Serializer& serializer) const
{
    serializer.insert(requested_count);
    
    for(unsigned int i=0; i < requested_count; i++)
        serializer.insert(requested_instances[i]);
    
}
void GetEventActionStatus::extract(Serializer& serializer)
{
    serializer.extract_count(requested_count, sizeof(requested_instances)/sizeof(requested_instances[0]));
    for(unsigned int i=0; i < requested_count; i++)
        serializer.extract(requested_instances[i]);
    
}

void GetEventActionStatus::Response::insert(Serializer& serializer) const
{
    serializer.insert(count);
    
    for(unsigned int i=0; i < count; i++)
        serializer.insert(actions[i]);
    
}
void GetEventActionStatus::Response::extract(Serializer& serializer)
{
    serializer.extract_count(count, sizeof(actions)/sizeof(actions[0]));
    for(unsigned int i=0; i < count; i++)
        serializer.extract(actions[i]);
    
}

TypedResult<GetEventActionStatus> getEventActionStatus(C::mip_interface& device, uint8_t requestedCount, const uint8_t* requestedInstances, uint8_t* countOut, uint8_t countOutMax, GetEventActionStatus::Entry* actionsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(requestedCount);
    
    assert(requestedInstances || (requestedCount == 0));
    for(unsigned int i=0; i < requestedCount; i++)
        serializer.insert(requestedInstances[i]);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GetEventActionStatus> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_STATUS, buffer, (uint8_t)serializer.usedLength(), REPLY_EVENT_ACTION_STATUS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*countOut, countOutMax);
        assert(actionsOut || (countOut == 0));
        for(unsigned int i=0; i < *countOut; i++)
            deserializer.extract(actionsOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void EventTrigger::GpioParams::insert(Serializer& serializer) const
{
    serializer.insert(pin);
    
    serializer.insert(mode);
    
}
void EventTrigger::GpioParams::extract(Serializer& serializer)
{
    serializer.extract(pin);
    
    serializer.extract(mode);
    
}

void EventTrigger::ThresholdParams::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(field_desc);
    
    serializer.insert(param_id);
    
    serializer.insert(type);
    
    if( type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        serializer.insert(low_thres);
        
    }
    if( type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        serializer.insert(int_thres);
        
    }
    if( type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        serializer.insert(high_thres);
        
    }
    if( type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        serializer.insert(interval);
        
    }
}
void EventTrigger::ThresholdParams::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract(field_desc);
    
    serializer.extract(param_id);
    
    serializer.extract(type);
    
    if( type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        serializer.extract(low_thres);
        
    }
    if( type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        serializer.extract(int_thres);
        
    }
    if( type == EventTrigger::ThresholdParams::Type::WINDOW )
    {
        serializer.extract(high_thres);
        
    }
    if( type == EventTrigger::ThresholdParams::Type::INTERVAL )
    {
        serializer.extract(interval);
        
    }
}

void EventTrigger::CombinationParams::insert(Serializer& serializer) const
{
    serializer.insert(logic_table);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(input_triggers[i]);
    
}
void EventTrigger::CombinationParams::extract(Serializer& serializer)
{
    serializer.extract(logic_table);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(input_triggers[i]);
    
}

void EventTrigger::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(instance);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(type);
        
        if( type == EventTrigger::Type::GPIO )
        {
            serializer.insert(parameters.gpio);
            
        }
        if( type == EventTrigger::Type::THRESHOLD )
        {
            serializer.insert(parameters.threshold);
            
        }
        if( type == EventTrigger::Type::COMBINATION )
        {
            serializer.insert(parameters.combination);
            
        }
    }
}
void EventTrigger::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(instance);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(type);
        
        if( type == EventTrigger::Type::GPIO )
        {
            serializer.extract(parameters.gpio);
            
        }
        if( type == EventTrigger::Type::THRESHOLD )
        {
            serializer.extract(parameters.threshold);
            
        }
        if( type == EventTrigger::Type::COMBINATION )
        {
            serializer.extract(parameters.combination);
            
        }
    }
}

void EventTrigger::Response::insert(Serializer& serializer) const
{
    serializer.insert(instance);
    
    serializer.insert(type);
    
    if( type == EventTrigger::Type::GPIO )
    {
        serializer.insert(parameters.gpio);
        
    }
    if( type == EventTrigger::Type::THRESHOLD )
    {
        serializer.insert(parameters.threshold);
        
    }
    if( type == EventTrigger::Type::COMBINATION )
    {
        serializer.insert(parameters.combination);
        
    }
}
void EventTrigger::Response::extract(Serializer& serializer)
{
    serializer.extract(instance);
    
    serializer.extract(type);
    
    if( type == EventTrigger::Type::GPIO )
    {
        serializer.extract(parameters.gpio);
        
    }
    if( type == EventTrigger::Type::THRESHOLD )
    {
        serializer.extract(parameters.threshold);
        
    }
    if( type == EventTrigger::Type::COMBINATION )
    {
        serializer.extract(parameters.combination);
        
    }
}

TypedResult<EventTrigger> writeEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type type, const EventTrigger::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(instance);
    
    serializer.insert(type);
    
    if( type == EventTrigger::Type::GPIO )
    {
        serializer.insert(parameters.gpio);
        
    }
    if( type == EventTrigger::Type::THRESHOLD )
    {
        serializer.insert(parameters.threshold);
        
    }
    if( type == EventTrigger::Type::COMBINATION )
    {
        serializer.insert(parameters.combination);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventTrigger> readEventTrigger(C::mip_interface& device, uint8_t instance, EventTrigger::Type* typeOut, EventTrigger::Parameters* parametersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EventTrigger> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)serializer.usedLength(), REPLY_EVENT_TRIGGER_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(instance);
        
        assert(typeOut);
        deserializer.extract(*typeOut);
        
        if( *typeOut == EventTrigger::Type::GPIO )
        {
            deserializer.extract(parametersOut->gpio);
            
        }
        if( *typeOut == EventTrigger::Type::THRESHOLD )
        {
            deserializer.extract(parametersOut->threshold);
            
        }
        if( *typeOut == EventTrigger::Type::COMBINATION )
        {
            deserializer.extract(parametersOut->combination);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EventTrigger> saveEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventTrigger> loadEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventTrigger> defaultEventTrigger(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_TRIGGER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
void EventAction::GpioParams::insert(Serializer& serializer) const
{
    serializer.insert(pin);
    
    serializer.insert(mode);
    
}
void EventAction::GpioParams::extract(Serializer& serializer)
{
    serializer.extract(pin);
    
    serializer.extract(mode);
    
}

void EventAction::MessageParams::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(decimation);
    
    serializer.insert(num_fields);
    
    for(unsigned int i=0; i < num_fields; i++)
        serializer.insert(descriptors[i]);
    
}
void EventAction::MessageParams::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract(decimation);
    
    serializer.extract_count(num_fields, sizeof(descriptors)/sizeof(descriptors[0]));
    for(unsigned int i=0; i < num_fields; i++)
        serializer.extract(descriptors[i]);
    
}

void EventAction::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(instance);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(trigger);
        
        serializer.insert(type);
        
        if( type == EventAction::Type::GPIO )
        {
            serializer.insert(parameters.gpio);
            
        }
        if( type == EventAction::Type::MESSAGE )
        {
            serializer.insert(parameters.message);
            
        }
    }
}
void EventAction::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(instance);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(trigger);
        
        serializer.extract(type);
        
        if( type == EventAction::Type::GPIO )
        {
            serializer.extract(parameters.gpio);
            
        }
        if( type == EventAction::Type::MESSAGE )
        {
            serializer.extract(parameters.message);
            
        }
    }
}

void EventAction::Response::insert(Serializer& serializer) const
{
    serializer.insert(instance);
    
    serializer.insert(trigger);
    
    serializer.insert(type);
    
    if( type == EventAction::Type::GPIO )
    {
        serializer.insert(parameters.gpio);
        
    }
    if( type == EventAction::Type::MESSAGE )
    {
        serializer.insert(parameters.message);
        
    }
}
void EventAction::Response::extract(Serializer& serializer)
{
    serializer.extract(instance);
    
    serializer.extract(trigger);
    
    serializer.extract(type);
    
    if( type == EventAction::Type::GPIO )
    {
        serializer.extract(parameters.gpio);
        
    }
    if( type == EventAction::Type::MESSAGE )
    {
        serializer.extract(parameters.message);
        
    }
}

TypedResult<EventAction> writeEventAction(C::mip_interface& device, uint8_t instance, uint8_t trigger, EventAction::Type type, const EventAction::Parameters& parameters)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(instance);
    
    serializer.insert(trigger);
    
    serializer.insert(type);
    
    if( type == EventAction::Type::GPIO )
    {
        serializer.insert(parameters.gpio);
        
    }
    if( type == EventAction::Type::MESSAGE )
    {
        serializer.insert(parameters.message);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventAction> readEventAction(C::mip_interface& device, uint8_t instance, uint8_t* triggerOut, EventAction::Type* typeOut, EventAction::Parameters* parametersOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EventAction> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)serializer.usedLength(), REPLY_EVENT_ACTION_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(instance);
        
        assert(triggerOut);
        deserializer.extract(*triggerOut);
        
        assert(typeOut);
        deserializer.extract(*typeOut);
        
        if( *typeOut == EventAction::Type::GPIO )
        {
            deserializer.extract(parametersOut->gpio);
            
        }
        if( *typeOut == EventAction::Type::MESSAGE )
        {
            deserializer.extract(parametersOut->message);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EventAction> saveEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventAction> loadEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<EventAction> defaultEventAction(C::mip_interface& device, uint8_t instance)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(instance);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EVENT_ACTION_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
void DeviceSettings::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
}
void DeviceSettings::extract(Serializer& serializer)
{
    serializer.extract(function);
    
}

TypedResult<DeviceSettings> saveDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<DeviceSettings> loadDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<DeviceSettings> defaultDeviceSettings(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DEVICE_STARTUP_SETTINGS, buffer, (uint8_t)serializer.usedLength());
}
void Sensor2VehicleTransformEuler::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(roll);
        
        serializer.insert(pitch);
        
        serializer.insert(yaw);
        
    }
}
void Sensor2VehicleTransformEuler::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(roll);
        
        serializer.extract(pitch);
        
        serializer.extract(yaw);
        
    }
}

void Sensor2VehicleTransformEuler::Response::insert(Serializer& serializer) const
{
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
}
void Sensor2VehicleTransformEuler::Response::extract(Serializer& serializer)
{
    serializer.extract(roll);
    
    serializer.extract(pitch);
    
    serializer.extract(yaw);
    
}

TypedResult<Sensor2VehicleTransformEuler> writeSensor2VehicleTransformEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformEuler> readSensor2VehicleTransformEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Sensor2VehicleTransformEuler> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rollOut);
        deserializer.extract(*rollOut);
        
        assert(pitchOut);
        deserializer.extract(*pitchOut);
        
        assert(yawOut);
        deserializer.extract(*yawOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Sensor2VehicleTransformEuler> saveSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformEuler> loadSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformEuler> defaultSensor2VehicleTransformEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_EUL, buffer, (uint8_t)serializer.usedLength());
}
void Sensor2VehicleTransformQuaternion::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(q);
        
    }
}
void Sensor2VehicleTransformQuaternion::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(q);
        
    }
}

void Sensor2VehicleTransformQuaternion::Response::insert(Serializer& serializer) const
{
    serializer.insert(q);
    
}
void Sensor2VehicleTransformQuaternion::Response::extract(Serializer& serializer)
{
    serializer.extract(q);
    
}

TypedResult<Sensor2VehicleTransformQuaternion> writeSensor2VehicleTransformQuaternion(C::mip_interface& device, const float* q)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(q);
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(q[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformQuaternion> readSensor2VehicleTransformQuaternion(C::mip_interface& device, float* qOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Sensor2VehicleTransformQuaternion> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(qOut);
        for(unsigned int i=0; i < 4; i++)
            deserializer.extract(qOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Sensor2VehicleTransformQuaternion> saveSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformQuaternion> loadSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformQuaternion> defaultSensor2VehicleTransformQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_QUAT, buffer, (uint8_t)serializer.usedLength());
}
void Sensor2VehicleTransformDcm::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(dcm);
        
    }
}
void Sensor2VehicleTransformDcm::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(dcm);
        
    }
}

void Sensor2VehicleTransformDcm::Response::insert(Serializer& serializer) const
{
    serializer.insert(dcm);
    
}
void Sensor2VehicleTransformDcm::Response::extract(Serializer& serializer)
{
    serializer.extract(dcm);
    
}

TypedResult<Sensor2VehicleTransformDcm> writeSensor2VehicleTransformDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(dcm);
    for(unsigned int i=0; i < 9; i++)
        serializer.insert(dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformDcm> readSensor2VehicleTransformDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Sensor2VehicleTransformDcm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(dcmOut);
        for(unsigned int i=0; i < 9; i++)
            deserializer.extract(dcmOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Sensor2VehicleTransformDcm> saveSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformDcm> loadSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Sensor2VehicleTransformDcm> defaultSensor2VehicleTransformDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_TRANSFORM_DCM, buffer, (uint8_t)serializer.usedLength());
}
void AccelBias::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(bias);
        
    }
}
void AccelBias::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(bias);
        
    }
}

void AccelBias::Response::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
}
void AccelBias::Response::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
}

TypedResult<AccelBias> writeAccelBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(bias);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(bias[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelBias> readAccelBias(C::mip_interface& device, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelBias> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)serializer.usedLength(), REPLY_ACCEL_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(biasOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelBias> saveAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelBias> loadAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<AccelBias> defaultAccelBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS, buffer, (uint8_t)serializer.usedLength());
}
void GyroBias::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(bias);
        
    }
}
void GyroBias::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(bias);
        
    }
}

void GyroBias::Response::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
}
void GyroBias::Response::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
}

TypedResult<GyroBias> writeGyroBias(C::mip_interface& device, const float* bias)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(bias);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(bias[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroBias> readGyroBias(C::mip_interface& device, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GyroBias> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)serializer.usedLength(), REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(biasOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GyroBias> saveGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroBias> loadGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GyroBias> defaultGyroBias(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS, buffer, (uint8_t)serializer.usedLength());
}
void CaptureGyroBias::insert(Serializer& serializer) const
{
    serializer.insert(averaging_time_ms);
    
}
void CaptureGyroBias::extract(Serializer& serializer)
{
    serializer.extract(averaging_time_ms);
    
}

void CaptureGyroBias::Response::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
}
void CaptureGyroBias::Response::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
}

TypedResult<CaptureGyroBias> captureGyroBias(C::mip_interface& device, uint16_t averagingTimeMs, float* biasOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(averagingTimeMs);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CaptureGyroBias> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CAPTURE_GYRO_BIAS, buffer, (uint8_t)serializer.usedLength(), REPLY_GYRO_BIAS_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(biasOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(biasOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void MagHardIronOffset::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(offset);
        
    }
}
void MagHardIronOffset::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(offset);
        
    }
}

void MagHardIronOffset::Response::insert(Serializer& serializer) const
{
    serializer.insert(offset);
    
}
void MagHardIronOffset::Response::extract(Serializer& serializer)
{
    serializer.extract(offset);
    
}

TypedResult<MagHardIronOffset> writeMagHardIronOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagHardIronOffset> readMagHardIronOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagHardIronOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)serializer.usedLength(), REPLY_HARD_IRON_OFFSET_VECTOR, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagHardIronOffset> saveMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagHardIronOffset> loadMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagHardIronOffset> defaultMagHardIronOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET, buffer, (uint8_t)serializer.usedLength());
}
void MagSoftIronMatrix::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(offset);
        
    }
}
void MagSoftIronMatrix::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(offset);
        
    }
}

void MagSoftIronMatrix::Response::insert(Serializer& serializer) const
{
    serializer.insert(offset);
    
}
void MagSoftIronMatrix::Response::extract(Serializer& serializer)
{
    serializer.extract(offset);
    
}

TypedResult<MagSoftIronMatrix> writeMagSoftIronMatrix(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    assert(offset);
    for(unsigned int i=0; i < 9; i++)
        serializer.insert(offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagSoftIronMatrix> readMagSoftIronMatrix(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagSoftIronMatrix> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)serializer.usedLength(), REPLY_SOFT_IRON_COMP_MATRIX, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut);
        for(unsigned int i=0; i < 9; i++)
            deserializer.extract(offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagSoftIronMatrix> saveMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagSoftIronMatrix> loadMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<MagSoftIronMatrix> defaultMagSoftIronMatrix(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX, buffer, (uint8_t)serializer.usedLength());
}
void ConingScullingEnable::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
    }
}
void ConingScullingEnable::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
    }
}

void ConingScullingEnable::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
}
void ConingScullingEnable::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
}

TypedResult<ConingScullingEnable> writeConingScullingEnable(C::mip_interface& device, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConingScullingEnable> readConingScullingEnable(C::mip_interface& device, bool* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ConingScullingEnable> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)serializer.usedLength(), REPLY_CONING_AND_SCULLING_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ConingScullingEnable> saveConingScullingEnable(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConingScullingEnable> loadConingScullingEnable(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ConingScullingEnable> defaultConingScullingEnable(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_CONING_AND_SCULLING_ENABLE, buffer, (uint8_t)serializer.usedLength());
}
void UartBaudrate::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(baud);
        
    }
}
void UartBaudrate::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(baud);
        
    }
}

void UartBaudrate::Response::insert(Serializer& serializer) const
{
    serializer.insert(baud);
    
}
void UartBaudrate::Response::extract(Serializer& serializer)
{
    serializer.extract(baud);
    
}

TypedResult<UartBaudrate> writeUartBaudrate(C::mip_interface& device, uint32_t baud)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(baud);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<UartBaudrate> readUartBaudrate(C::mip_interface& device, uint32_t* baudOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<UartBaudrate> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)serializer.usedLength(), REPLY_UART_BAUDRATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(baudOut);
        deserializer.extract(*baudOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<UartBaudrate> saveUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<UartBaudrate> loadUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<UartBaudrate> defaultUartBaudrate(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_UART_BAUDRATE, buffer, (uint8_t)serializer.usedLength());
}
void GpioConfig::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(pin);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(feature);
        
        serializer.insert(behavior);
        
        serializer.insert(pin_mode);
        
    }
}
void GpioConfig::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(pin);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(feature);
        
        serializer.extract(behavior);
        
        serializer.extract(pin_mode);
        
    }
}

void GpioConfig::Response::insert(Serializer& serializer) const
{
    serializer.insert(pin);
    
    serializer.insert(feature);
    
    serializer.insert(behavior);
    
    serializer.insert(pin_mode);
    
}
void GpioConfig::Response::extract(Serializer& serializer)
{
    serializer.extract(pin);
    
    serializer.extract(feature);
    
    serializer.extract(behavior);
    
    serializer.extract(pin_mode);
    
}

TypedResult<GpioConfig> writeGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature feature, GpioConfig::Behavior behavior, GpioConfig::PinMode pinMode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(pin);
    
    serializer.insert(feature);
    
    serializer.insert(behavior);
    
    serializer.insert(pinMode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GpioConfig> readGpioConfig(C::mip_interface& device, uint8_t pin, GpioConfig::Feature* featureOut, GpioConfig::Behavior* behaviorOut, GpioConfig::PinMode* pinModeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(pin);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GpioConfig> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)serializer.usedLength(), REPLY_GPIO_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(pin);
        
        assert(featureOut);
        deserializer.extract(*featureOut);
        
        assert(behaviorOut);
        deserializer.extract(*behaviorOut);
        
        assert(pinModeOut);
        deserializer.extract(*pinModeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GpioConfig> saveGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GpioConfig> loadGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GpioConfig> defaultGpioConfig(C::mip_interface& device, uint8_t pin)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(pin);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
void GpioState::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE || function == FunctionSelector::READ )
    {
        serializer.insert(pin);
        
    }
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(state);
        
    }
}
void GpioState::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE || function == FunctionSelector::READ )
    {
        serializer.extract(pin);
        
    }
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(state);
        
    }
}

void GpioState::Response::insert(Serializer& serializer) const
{
    serializer.insert(pin);
    
    serializer.insert(state);
    
}
void GpioState::Response::extract(Serializer& serializer)
{
    serializer.extract(pin);
    
    serializer.extract(state);
    
}

TypedResult<GpioState> writeGpioState(C::mip_interface& device, uint8_t pin, bool state)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(pin);
    
    serializer.insert(state);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<GpioState> readGpioState(C::mip_interface& device, uint8_t pin, bool* stateOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(pin);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GpioState> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GPIO_STATE, buffer, (uint8_t)serializer.usedLength(), REPLY_GPIO_STATE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(pin);
        
        assert(stateOut);
        deserializer.extract(*stateOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void Odometer::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(mode);
        
        serializer.insert(scaling);
        
        serializer.insert(uncertainty);
        
    }
}
void Odometer::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(mode);
        
        serializer.extract(scaling);
        
        serializer.extract(uncertainty);
        
    }
}

void Odometer::Response::insert(Serializer& serializer) const
{
    serializer.insert(mode);
    
    serializer.insert(scaling);
    
    serializer.insert(uncertainty);
    
}
void Odometer::Response::extract(Serializer& serializer)
{
    serializer.extract(mode);
    
    serializer.extract(scaling);
    
    serializer.extract(uncertainty);
    
}

TypedResult<Odometer> writeOdometer(C::mip_interface& device, Odometer::Mode mode, float scaling, float uncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(mode);
    
    serializer.insert(scaling);
    
    serializer.insert(uncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Odometer> readOdometer(C::mip_interface& device, Odometer::Mode* modeOut, float* scalingOut, float* uncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<Odometer> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)serializer.usedLength(), REPLY_ODOMETER_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        deserializer.extract(*modeOut);
        
        assert(scalingOut);
        deserializer.extract(*scalingOut);
        
        assert(uncertaintyOut);
        deserializer.extract(*uncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<Odometer> saveOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Odometer> loadOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<Odometer> defaultOdometer(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ODOMETER_CONFIG, buffer, (uint8_t)serializer.usedLength());
}
void ImuLowpassFilter::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(target_descriptor);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(manual);
        
        serializer.insert(frequency);
        
        serializer.insert(reserved);
        
    }
}
void ImuLowpassFilter::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(target_descriptor);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(manual);
        
        serializer.extract(frequency);
        
        serializer.extract(reserved);
        
    }
}

void ImuLowpassFilter::Response::insert(Serializer& serializer) const
{
    serializer.insert(target_descriptor);
    
    serializer.insert(enable);
    
    serializer.insert(manual);
    
    serializer.insert(frequency);
    
    serializer.insert(reserved);
    
}
void ImuLowpassFilter::Response::extract(Serializer& serializer)
{
    serializer.extract(target_descriptor);
    
    serializer.extract(enable);
    
    serializer.extract(manual);
    
    serializer.extract(frequency);
    
    serializer.extract(reserved);
    
}

TypedResult<ImuLowpassFilter> writeImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool enable, bool manual, uint16_t frequency, uint8_t reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(targetDescriptor);
    
    serializer.insert(enable);
    
    serializer.insert(manual);
    
    serializer.insert(frequency);
    
    serializer.insert(reserved);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ImuLowpassFilter> readImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor, bool* enableOut, bool* manualOut, uint16_t* frequencyOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(targetDescriptor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ImuLowpassFilter> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength(), REPLY_ADVANCED_DATA_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(targetDescriptor);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(manualOut);
        deserializer.extract(*manualOut);
        
        assert(frequencyOut);
        deserializer.extract(*frequencyOut);
        
        assert(reservedOut);
        deserializer.extract(*reservedOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ImuLowpassFilter> saveImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ImuLowpassFilter> loadImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ImuLowpassFilter> defaultImuLowpassFilter(C::mip_interface& device, uint8_t targetDescriptor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(targetDescriptor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_IMU_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
void ComplementaryFilter::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(pitch_roll_enable);
        
        serializer.insert(heading_enable);
        
        serializer.insert(pitch_roll_time_constant);
        
        serializer.insert(heading_time_constant);
        
    }
}
void ComplementaryFilter::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(pitch_roll_enable);
        
        serializer.extract(heading_enable);
        
        serializer.extract(pitch_roll_time_constant);
        
        serializer.extract(heading_time_constant);
        
    }
}

void ComplementaryFilter::Response::insert(Serializer& serializer) const
{
    serializer.insert(pitch_roll_enable);
    
    serializer.insert(heading_enable);
    
    serializer.insert(pitch_roll_time_constant);
    
    serializer.insert(heading_time_constant);
    
}
void ComplementaryFilter::Response::extract(Serializer& serializer)
{
    serializer.extract(pitch_roll_enable);
    
    serializer.extract(heading_enable);
    
    serializer.extract(pitch_roll_time_constant);
    
    serializer.extract(heading_time_constant);
    
}

TypedResult<ComplementaryFilter> writeComplementaryFilter(C::mip_interface& device, bool pitchRollEnable, bool headingEnable, float pitchRollTimeConstant, float headingTimeConstant)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(pitchRollEnable);
    
    serializer.insert(headingEnable);
    
    serializer.insert(pitchRollTimeConstant);
    
    serializer.insert(headingTimeConstant);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ComplementaryFilter> readComplementaryFilter(C::mip_interface& device, bool* pitchRollEnableOut, bool* headingEnableOut, float* pitchRollTimeConstantOut, float* headingTimeConstantOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ComplementaryFilter> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)serializer.usedLength(), REPLY_LEGACY_COMP_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(pitchRollEnableOut);
        deserializer.extract(*pitchRollEnableOut);
        
        assert(headingEnableOut);
        deserializer.extract(*headingEnableOut);
        
        assert(pitchRollTimeConstantOut);
        deserializer.extract(*pitchRollTimeConstantOut);
        
        assert(headingTimeConstantOut);
        deserializer.extract(*headingTimeConstantOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ComplementaryFilter> saveComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ComplementaryFilter> loadComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<ComplementaryFilter> defaultComplementaryFilter(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LEGACY_COMP_FILTER, buffer, (uint8_t)serializer.usedLength());
}
void SensorRange::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(sensor);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(setting);
        
    }
}
void SensorRange::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(sensor);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(setting);
        
    }
}

void SensorRange::Response::insert(Serializer& serializer) const
{
    serializer.insert(sensor);
    
    serializer.insert(setting);
    
}
void SensorRange::Response::extract(Serializer& serializer)
{
    serializer.extract(sensor);
    
    serializer.extract(setting);
    
}

TypedResult<SensorRange> writeSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t setting)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(sensor);
    
    serializer.insert(setting);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorRange> readSensorRange(C::mip_interface& device, SensorRangeType sensor, uint8_t* settingOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(sensor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorRange> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)serializer.usedLength(), REPLY_SENSOR_RANGE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(sensor);
        
        assert(settingOut);
        deserializer.extract(*settingOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorRange> saveSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorRange> loadSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SensorRange> defaultSensorRange(C::mip_interface& device, SensorRangeType sensor)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(sensor);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR_RANGE, buffer, (uint8_t)serializer.usedLength());
}
void CalibratedSensorRanges::Entry::insert(Serializer& serializer) const
{
    serializer.insert(setting);
    
    serializer.insert(range);
    
}
void CalibratedSensorRanges::Entry::extract(Serializer& serializer)
{
    serializer.extract(setting);
    
    serializer.extract(range);
    
}

void CalibratedSensorRanges::insert(Serializer& serializer) const
{
    serializer.insert(sensor);
    
}
void CalibratedSensorRanges::extract(Serializer& serializer)
{
    serializer.extract(sensor);
    
}

void CalibratedSensorRanges::Response::insert(Serializer& serializer) const
{
    serializer.insert(sensor);
    
    serializer.insert(num_ranges);
    
    for(unsigned int i=0; i < num_ranges; i++)
        serializer.insert(ranges[i]);
    
}
void CalibratedSensorRanges::Response::extract(Serializer& serializer)
{
    serializer.extract(sensor);
    
    serializer.extract_count(num_ranges, sizeof(ranges)/sizeof(ranges[0]));
    for(unsigned int i=0; i < num_ranges; i++)
        serializer.extract(ranges[i]);
    
}

TypedResult<CalibratedSensorRanges> calibratedSensorRanges(C::mip_interface& device, SensorRangeType sensor, uint8_t* numRangesOut, uint8_t numRangesOutMax, CalibratedSensorRanges::Entry* rangesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(sensor);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CalibratedSensorRanges> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_CALIBRATED_RANGES, buffer, (uint8_t)serializer.usedLength(), REPLY_CALIBRATED_RANGES, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(sensor);
        
        deserializer.extract_count(*numRangesOut, numRangesOutMax);
        assert(rangesOut || (numRangesOut == 0));
        for(unsigned int i=0; i < *numRangesOut; i++)
            deserializer.extract(rangesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void LowpassFilter::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(desc_set);
    
    serializer.insert(field_desc);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(manual);
        
        serializer.insert(frequency);
        
    }
}
void LowpassFilter::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(desc_set);
    
    serializer.extract(field_desc);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(manual);
        
        serializer.extract(frequency);
        
    }
}

void LowpassFilter::Response::insert(Serializer& serializer) const
{
    serializer.insert(desc_set);
    
    serializer.insert(field_desc);
    
    serializer.insert(enable);
    
    serializer.insert(manual);
    
    serializer.insert(frequency);
    
}
void LowpassFilter::Response::extract(Serializer& serializer)
{
    serializer.extract(desc_set);
    
    serializer.extract(field_desc);
    
    serializer.extract(enable);
    
    serializer.extract(manual);
    
    serializer.extract(frequency);
    
}

TypedResult<LowpassFilter> writeLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc, bool enable, bool manual, float frequency)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(descSet);
    
    serializer.insert(fieldDesc);
    
    serializer.insert(enable);
    
    serializer.insert(manual);
    
    serializer.insert(frequency);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<LowpassFilter> readLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc, bool* enableOut, bool* manualOut, float* frequencyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(descSet);
    
    serializer.insert(fieldDesc);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<LowpassFilter> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength(), REPLY_LOWPASS_FILTER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(descSet);
        
        deserializer.extract(fieldDesc);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(manualOut);
        deserializer.extract(*manualOut);
        
        assert(frequencyOut);
        deserializer.extract(*frequencyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<LowpassFilter> saveLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(descSet);
    
    serializer.insert(fieldDesc);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<LowpassFilter> loadLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(descSet);
    
    serializer.insert(fieldDesc);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<LowpassFilter> defaultLowpassFilter(C::mip_interface& device, uint8_t descSet, uint8_t fieldDesc)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(descSet);
    
    serializer.insert(fieldDesc);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_LOWPASS_FILTER, buffer, (uint8_t)serializer.usedLength());
}

} // namespace commands_3dm
} // namespace mip

