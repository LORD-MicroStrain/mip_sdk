
#include "commands_system.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace commands_system {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void CommMode::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(mode);
        
    }
}
void CommMode::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(mode);
        
    }
}

void CommMode::Response::insert(Serializer& serializer) const
{
    serializer.insert(mode);
    
}
void CommMode::Response::extract(Serializer& serializer)
{
    serializer.extract(mode);
    
}

TypedResult<CommMode> writeCommMode(C::mip_interface& device, uint8_t mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<CommMode> readCommMode(C::mip_interface& device, uint8_t* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CommMode> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, (uint8_t)serializer.usedLength(), REPLY_COM_MODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        deserializer.extract(*modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<CommMode> defaultCommMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, (uint8_t)serializer.usedLength());
}
void InterfaceControl::insert(Serializer& serializer) const
{
    serializer.insert(function);
    
    serializer.insert(interface);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(protocols_in);
        
        serializer.insert(protocols_out);
        
    }
}
void InterfaceControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(interface);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(protocols_in);
        
        serializer.extract(protocols_out);
        
    }
}

void InterfaceControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(interface);
    
    serializer.insert(protocols_in);
    
    serializer.insert(protocols_out);
    
}
void InterfaceControl::Response::extract(Serializer& serializer)
{
    serializer.extract(interface);
    
    serializer.extract(protocols_in);
    
    serializer.extract(protocols_out);
    
}

TypedResult<InterfaceControl> writeInterfaceControl(C::mip_interface& device, CommsInterface interface, CommsProtocol protocolsIn, CommsProtocol protocolsOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(interface);
    
    serializer.insert(protocolsIn);
    
    serializer.insert(protocolsOut);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InterfaceControl> readInterfaceControl(C::mip_interface& device, CommsInterface interface, CommsProtocol* protocolsInOut, CommsProtocol* protocolsOutOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(interface);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<InterfaceControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_INTERFACE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(interface);
        
        assert(protocolsInOut);
        deserializer.extract(*protocolsInOut);
        
        assert(protocolsOutOut);
        deserializer.extract(*protocolsOutOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<InterfaceControl> saveInterfaceControl(C::mip_interface& device, CommsInterface interface)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(interface);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InterfaceControl> loadInterfaceControl(C::mip_interface& device, CommsInterface interface)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(interface);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InterfaceControl> defaultInterfaceControl(C::mip_interface& device, CommsInterface interface)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(interface);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}

} // namespace commands_system
} // namespace mip

