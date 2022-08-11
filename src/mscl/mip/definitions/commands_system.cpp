
#include "commands_system.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class MipSerializer;

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

void insert(MipSerializer& serializer, const CommMode& self)
{
    insert(serializer, self.function);
    insert(serializer, self.mode);
}

void extract(MipSerializer& serializer, CommMode& self)
{
    extract(serializer, self.function);
    extract(serializer, self.mode);
}

/// @brief Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
/// @param mode 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult writeCommMode(C::mip_interface& device, uint8_t mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::WRITE);
    insert(serializer, mode);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, serializer.offset);
}

/// @brief Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
/// @param[out] mode 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult readCommMode(C::mip_interface& device, uint8_t& mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::READ);
    assert(!!serializer);
    
    uint8_t responseLength;
    mip_cmd_result result_local = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, serializer.offset, REPLY_COM_MODE, buffer, &responseLength);
    
    if( result_local == MIP_ACK_OK )
    {
        MipSerializer serializer(buffer, sizeof(buffer));
        
        extract(serializer, mode);
        
        if( !!!serializer )
            result_local = MIP_STATUS_ERROR;
    }
    return result_local;
}

/// @brief Advanced specialized communication modes.
/// 
/// This command allows the user to communicate directly with various subsystems which may be present in MIP devices (i.e. IMU, GNSS, etc.)
/// Please see the specific device's user manual for possible modes.
/// 
/// This command responds with an ACK/NACK just prior to switching to the new protocol.
/// For all functions except 0x01 (use new settings), the new communications mode value is ignored.
/// 
/// 
/// 
/// @returns MipCmdResult
/// 
MipCmdResult defaultCommMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    MipSerializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, MipFunctionSelector::RESET);
    assert(!!serializer);
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COM_MODE, buffer, serializer.offset);
}


} // namespace commands_system
} // namespace mip

