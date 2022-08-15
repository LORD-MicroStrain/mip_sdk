
#include "commands_gnss.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_gnss {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}

void insert(Serializer& serializer, const ReceiverInfo::Info& self)
{
    insert(serializer, self.receiver_id);
    
    insert(serializer, self.mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        insert(serializer, self.description[i]);
    
}
void extract(Serializer& serializer, ReceiverInfo::Info& self)
{
    extract(serializer, self.receiver_id);
    
    extract(serializer, self.mip_data_descriptor_set);
    
    for(unsigned int i=0; i < 32; i++)
        extract(serializer, self.description[i]);
    
}

CmdResult receiverInfo(C::mip_interface& device, uint8_t* numReceiversOut, uint8_t numReceiversOutMax, ReceiverInfo::Info* receiverInfoOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_LIST_RECEIVERS, NULL, 0, REPLY_LIST_RECEIVERS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        C::extract_count(&deserializer, numReceiversOut, numReceiversOutMax);
        assert(receiverInfoOut);
        assert(numReceiversOut);
        for(unsigned int i=0; i < *numReceiversOut; i++)
            extract(deserializer, receiverInfoOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void insert(Serializer& serializer, const SignalConfiguration& self)
{
    insert(serializer, self.gps_enable);
    
    insert(serializer, self.glonass_enable);
    
    insert(serializer, self.galileo_enable);
    
    insert(serializer, self.beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.reserved[i]);
    
}
void extract(Serializer& serializer, SignalConfiguration& self)
{
    extract(serializer, self.gps_enable);
    
    extract(serializer, self.glonass_enable);
    
    extract(serializer, self.galileo_enable);
    
    extract(serializer, self.beidou_enable);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.reserved[i]);
    
}

CmdResult writeSignalConfiguration(C::mip_interface& device, uint8_t gpsEnable, uint8_t glonassEnable, uint8_t galileoEnable, uint8_t beidouEnable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, gpsEnable);
    
    insert(serializer, glonassEnable);
    
    insert(serializer, galileoEnable);
    
    insert(serializer, beidouEnable);
    
    assert(reserved);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, reserved[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
CmdResult readSignalConfiguration(C::mip_interface& device, uint8_t* gpsEnableOut, uint8_t* glonassEnableOut, uint8_t* galileoEnableOut, uint8_t* beidouEnableOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset, REPLY_SIGNAL_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(gpsEnableOut);
        extract(deserializer, *gpsEnableOut);
        
        assert(glonassEnableOut);
        extract(deserializer, *glonassEnableOut);
        
        assert(galileoEnableOut);
        extract(deserializer, *galileoEnableOut);
        
        assert(beidouEnableOut);
        extract(deserializer, *beidouEnableOut);
        
        assert(reservedOut);
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, reservedOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
CmdResult loadSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
CmdResult defaultSignalConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SIGNAL_CONFIGURATION, buffer, serializer.offset);
}
void insert(Serializer& serializer, const RtkDongleConfiguration& self)
{
    insert(serializer, self.enable);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reserved[i]);
    
}
void extract(Serializer& serializer, RtkDongleConfiguration& self)
{
    extract(serializer, self.enable);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reserved[i]);
    
}

CmdResult writeRtkDongleConfiguration(C::mip_interface& device, uint8_t enable, const uint8_t* reserved)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(reserved);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, reserved[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
CmdResult readRtkDongleConfiguration(C::mip_interface& device, uint8_t* enableOut, uint8_t* reservedOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength;
    CmdResult result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset, REPLY_RTK_DONGLE_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, sizeof(buffer));
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(reservedOut);
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, reservedOut[i]);
        
        if( !deserializer.isOk() )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
CmdResult saveRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
CmdResult loadRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
CmdResult defaultRtkDongleConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RTK_DONGLE_CONFIGURATION, buffer, serializer.offset);
}
void insert(Serializer& serializer, const ReceiverSafeMode& self)
{
    insert(serializer, self.receiver_id);
    
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, ReceiverSafeMode& self)
{
    extract(serializer, self.receiver_id);
    
    extract(serializer, self.enable);
    
}

CmdResult receiverSafeMode(C::mip_interface& device, uint8_t receiverId, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, receiverId);
    
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RECEIVER_SAFE_MODE, buffer, serializer.offset);
}

} // namespace commands_gnss
} // namespace mip

