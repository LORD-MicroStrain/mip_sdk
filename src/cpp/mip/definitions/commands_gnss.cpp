
#include "commands_gnss.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_gnss {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void ReceiverInfo::Info::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(description[i]);
    
}
void ReceiverInfo::Info::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(description[i]);
    
}

void ReceiverInfo::insert(Serializer& serializer) const
{
    (void)serializer;
}
void ReceiverInfo::extract(Serializer& serializer)
{
    (void)serializer;
}

void ReceiverInfo::Response::insert(Serializer& serializer) const
{
    serializer.insert(num_receivers);
    
    for(unsigned int i=0; i < num_receivers; i++)
        serializer.insert(receiver_info[i]);
    
}
void ReceiverInfo::Response::extract(Serializer& serializer)
{
    serializer.extract_count(num_receivers, sizeof(receiver_info)/sizeof(receiver_info[0]));
    for(unsigned int i=0; i < num_receivers; i++)
        serializer.extract(receiver_info[i]);
    
}

TypedResult<ReceiverInfo> receiverInfo(C::mip_interface& device, uint8_t* numReceiversOut, uint8_t numReceiversOutMax, ReceiverInfo::Info* receiverInfoOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<ReceiverInfo> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LIST_RECEIVERS, NULL, 0, REPLY_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract_count(*numReceiversOut, numReceiversOutMax);
        assert(receiverInfoOut || (numReceiversOut == 0));
        for(unsigned int i=0; i < *numReceiversOut; i++)
            deserializer.extract(receiverInfoOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void SignalConfiguration::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(gps_enable);
        
        serializer.insert(glonass_enable);
        
        serializer.insert(galileo_enable);
        
        serializer.insert(beidou_enable);
        
        for(unsigned int i=0; i < 4; i++)
            serializer.insert(reserved[i]);
        
    }
}
void SignalConfiguration::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(gps_enable);
        
        serializer.extract(glonass_enable);
        
        serializer.extract(galileo_enable);
        
        serializer.extract(beidou_enable);
        
        for(unsigned int i=0; i < 4; i++)
            serializer.extract(reserved[i]);
        
    }
}

void SignalConfiguration::Response::insert(Serializer& serializer) const
{
    serializer.insert(gps_enable);
    
    serializer.insert(glonass_enable);
    
    serializer.insert(galileo_enable);
    
    serializer.insert(beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(reserved[i]);
    
}
void SignalConfiguration::Response::extract(Serializer& serializer)
{
    serializer.extract(gps_enable);
    
    serializer.extract(glonass_enable);
    
    serializer.extract(galileo_enable);
    
    serializer.extract(beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(reserved[i]);
    
}

TypedResult<SignalConfiguration> writeSignalConfiguration(C::mip_interface& device, uint8_t gpsEnable, uint8_t glonassEnable, uint8_t galileoEnable, uint8_t beidouEnable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(gpsEnable);
    
    serializer.insert(glonassEnable);
    
    serializer.insert(galileoEnable);
    
    serializer.insert(beidouEnable);
    
    assert(reserved);
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(reserved[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SignalConfiguration> readSignalConfiguration(C::mip_interface& device, uint8_t* gpsEnableOut, uint8_t* glonassEnableOut, uint8_t* galileoEnableOut, uint8_t* beidouEnableOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SignalConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)serializer.usedLength(), REPLY_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(gpsEnableOut);
        deserializer.extract(*gpsEnableOut);
        
        assert(glonassEnableOut);
        deserializer.extract(*glonassEnableOut);
        
        assert(galileoEnableOut);
        deserializer.extract(*galileoEnableOut);
        
        assert(beidouEnableOut);
        deserializer.extract(*beidouEnableOut);
        
        assert(reservedOut);
        for(unsigned int i=0; i < 4; i++)
            deserializer.extract(reservedOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SignalConfiguration> saveSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SignalConfiguration> loadSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SignalConfiguration> defaultSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
void SpartnConfiguration::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        serializer.insert(type);
        
        serializer.insert(current_key_tow);
        
        serializer.insert(current_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            serializer.insert(current_key[i]);
        
        serializer.insert(next_key_tow);
        
        serializer.insert(next_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            serializer.insert(next_key[i]);
        
    }
}
void SpartnConfiguration::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        serializer.extract(type);
        
        serializer.extract(current_key_tow);
        
        serializer.extract(current_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            serializer.extract(current_key[i]);
        
        serializer.extract(next_key_tow);
        
        serializer.extract(next_key_week);
        
        for(unsigned int i=0; i < 32; i++)
            serializer.extract(next_key[i]);
        
    }
}

void SpartnConfiguration::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    serializer.insert(type);
    
    serializer.insert(current_key_tow);
    
    serializer.insert(current_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(current_key[i]);
    
    serializer.insert(next_key_tow);
    
    serializer.insert(next_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(next_key[i]);
    
}
void SpartnConfiguration::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    serializer.extract(type);
    
    serializer.extract(current_key_tow);
    
    serializer.extract(current_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(current_key[i]);
    
    serializer.extract(next_key_tow);
    
    serializer.extract(next_key_week);
    
    for(unsigned int i=0; i < 32; i++)
        serializer.extract(next_key[i]);
    
}

TypedResult<SpartnConfiguration> writeSpartnConfiguration(C::mip_interface& device, uint8_t enable, uint8_t type, uint32_t currentKeyTow, uint16_t currentKeyWeek, const uint8_t* currentKey, uint32_t nextKeyTow, uint16_t nextKeyWeek, const uint8_t* nextKey)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    serializer.insert(type);
    
    serializer.insert(currentKeyTow);
    
    serializer.insert(currentKeyWeek);
    
    assert(currentKey);
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(currentKey[i]);
    
    serializer.insert(nextKeyTow);
    
    serializer.insert(nextKeyWeek);
    
    assert(nextKey);
    for(unsigned int i=0; i < 32; i++)
        serializer.insert(nextKey[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPARTN_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SpartnConfiguration> readSpartnConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* typeOut, uint32_t* currentKeyTowOut, uint16_t* currentKeyWeekOut, uint8_t* currentKeyOut, uint32_t* nextKeyTowOut, uint16_t* nextKeyWeekOut, uint8_t* nextKeyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SpartnConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SPARTN_CONFIGURATION, buffer, (uint8_t)serializer.usedLength(), REPLY_SPARTN_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(typeOut);
        deserializer.extract(*typeOut);
        
        assert(currentKeyTowOut);
        deserializer.extract(*currentKeyTowOut);
        
        assert(currentKeyWeekOut);
        deserializer.extract(*currentKeyWeekOut);
        
        assert(currentKeyOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(currentKeyOut[i]);
        
        assert(nextKeyTowOut);
        deserializer.extract(*nextKeyTowOut);
        
        assert(nextKeyWeekOut);
        deserializer.extract(*nextKeyWeekOut);
        
        assert(nextKeyOut);
        for(unsigned int i=0; i < 32; i++)
            deserializer.extract(nextKeyOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SpartnConfiguration> saveSpartnConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPARTN_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SpartnConfiguration> loadSpartnConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPARTN_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<SpartnConfiguration> defaultSpartnConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPARTN_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
void RtkDongleConfiguration::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(enable);
        
        for(unsigned int i=0; i < 3; i++)
            serializer.insert(reserved[i]);
        
    }
}
void RtkDongleConfiguration::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(enable);
        
        for(unsigned int i=0; i < 3; i++)
            serializer.extract(reserved[i]);
        
    }
}

void RtkDongleConfiguration::Response::insert(Serializer& serializer) const
{
    serializer.insert(enable);
    
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(reserved[i]);
    
}
void RtkDongleConfiguration::Response::extract(Serializer& serializer)
{
    serializer.extract(enable);
    
    for(unsigned int i=0; i < 3; i++)
        serializer.extract(reserved[i]);
    
}

TypedResult<RtkDongleConfiguration> writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(enable);
    
    assert(reserved);
    for(unsigned int i=0; i < 3; i++)
        serializer.insert(reserved[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RtkDongleConfiguration> readRtkDongleConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<RtkDongleConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)serializer.usedLength(), REPLY_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        deserializer.extract(*enableOut);
        
        assert(reservedOut);
        for(unsigned int i=0; i < 3; i++)
            deserializer.extract(reservedOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<RtkDongleConfiguration> saveRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RtkDongleConfiguration> loadRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<RtkDongleConfiguration> defaultRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, (uint8_t)serializer.usedLength());
}

} // namespace commands_gnss
} // namespace mip

