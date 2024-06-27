
#include "commands_system.hpp"

#include "microstrain/common/serialization.hpp"
#include "../mip_interface.hpp"

#include <assert.h>


namespace mip {
;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_system {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(::microstrain::Serializer& serializer, const CommMode& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
    }
}
void extract(::microstrain::Serializer& serializer, CommMode& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
    }
}

void insert(::microstrain::Serializer& serializer, const CommMode::Response& self)
{
    insert(serializer, self.mode);
    
}
void extract(::microstrain::Serializer& serializer, CommMode::Response& self)
{
    extract(serializer, self.mode);
    
}

TypedResult<CommMode> writeCommMode(C::mip_interface& device, uint8_t mode)
{
    uint8_t                 buffer[C::MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain::Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, (uint8_t)serializer.length());
}
TypedResult<CommMode> readCommMode(C::mip_interface& device, uint8_t* modeOut)
{
    uint8_t                 buffer[C::MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain::Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<CommMode> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, (uint8_t)serializer.length(), REPLY_COM_MODE, buffer, &responseLength);
    
    if( result == CmdResult::ACK_OK )
    {
        microstrain::Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        if( deserializer.remaining() != 0 )
            result = CmdResult::STATUS_ERROR;
    }
    return result;
}
TypedResult<CommMode> defaultCommMode(C::mip_interface& device)
{
    uint8_t                 buffer[C::MIP_FIELD_PAYLOAD_LENGTH_MAX];
    microstrain::Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, (uint8_t)serializer.length());
}

} // namespace commands_system
} // namespace mip
