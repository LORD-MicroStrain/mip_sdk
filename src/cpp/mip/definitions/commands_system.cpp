
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
    
    serializer.insert(port);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.insert(protocols_incoming);
        
        serializer.insert(protocols_outgoing);
        
    }
}
void InterfaceControl::extract(Serializer& serializer)
{
    serializer.extract(function);
    
    serializer.extract(port);
    
    if( function == FunctionSelector::WRITE )
    {
        serializer.extract(protocols_incoming);
        
        serializer.extract(protocols_outgoing);
        
    }
}

void InterfaceControl::Response::insert(Serializer& serializer) const
{
    serializer.insert(port);
    
    serializer.insert(protocols_incoming);
    
    serializer.insert(protocols_outgoing);
    
}
void InterfaceControl::Response::extract(Serializer& serializer)
{
    serializer.extract(port);
    
    serializer.extract(protocols_incoming);
    
    serializer.extract(protocols_outgoing);
    
}

TypedResult<InterfaceControl> writeInterfaceControl(C::mip_interface& device, CommsInterface port, CommsProtocol protocolsIncoming, CommsProtocol protocolsOutgoing)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::WRITE);
    serializer.insert(port);
    
    serializer.insert(protocolsIncoming);
    
    serializer.insert(protocolsOutgoing);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InterfaceControl> readInterfaceControl(C::mip_interface& device, CommsInterface port, CommsProtocol* protocolsIncomingOut, CommsProtocol* protocolsOutgoingOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::READ);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<InterfaceControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength(), REPLY_INTERFACE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        deserializer.extract(port);
        
        assert(protocolsIncomingOut);
        deserializer.extract(*protocolsIncomingOut);
        
        assert(protocolsOutgoingOut);
        deserializer.extract(*protocolsOutgoingOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<InterfaceControl> saveInterfaceControl(C::mip_interface& device, CommsInterface port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::SAVE);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InterfaceControl> loadInterfaceControl(C::mip_interface& device, CommsInterface port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::LOAD);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}
TypedResult<InterfaceControl> defaultInterfaceControl(C::mip_interface& device, CommsInterface port)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    serializer.insert(FunctionSelector::RESET);
    serializer.insert(port);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INTERFACE_CONTROL, buffer, (uint8_t)serializer.usedLength());
}

} // namespace commands_system
} // namespace mip

