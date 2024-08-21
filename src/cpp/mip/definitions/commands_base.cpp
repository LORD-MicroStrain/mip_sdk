
#include "commands_base.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_base {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void BaseDeviceInfo::insert(Serializer& serializer) const
{
    serializer.insert(firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(device_options[i]);
    
}
void BaseDeviceInfo::extract(Serializer& serializer)
{
    serializer.extract(firmware_version);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(model_name[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(model_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(serial_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(lot_number[i]);
    
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(device_options[i]);
    
}

void Ping::insert(Serializer& serializer) const
{
    (void)serializer;
}
void Ping::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<Ping> ping(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PING, NULL, 0);
}
void SetIdle::insert(Serializer& serializer) const
{
    (void)serializer;
}
void SetIdle::extract(Serializer& serializer)
{
    (void)serializer;
}

TypedResult<SetIdle> setIdle(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_TO_IDLE, NULL, 0);
}
void GetDeviceInfo::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetDeviceInfo::extract(Serializer& serializer)
{
    (void)serializer;
}

void GetDeviceInfo::Response::insert(Serializer& serializer) const
{
    serializer.insert(device_info);
    
}
void GetDeviceInfo::Response::extract(Serializer& serializer)
{
    serializer.extract(device_info);
    
}

TypedResult<GetDeviceInfo> getDeviceInfo(C::mip_interface& device, BaseDeviceInfo* deviceInfoOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    uint8_t responseLength = sizeof(buffer);
    
    TypedResult<GetDeviceInfo> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GET_DEVICE_INFO, NULL, 0, REPLY_DEVICE_INFO, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(deviceInfoOut);
        deserializer.extract(*deviceInfoOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
void GetDeviceDescriptors::insert(Serializer& serializer) const
{
    (void)serializer;
}
void GetDeviceDescriptors::extract(Serializer& serializer)
{
    (void)serializer;
}

