
#include "mip_cmd_utils.hpp"


namespace mip
{

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the device to idle.
///
/// This is the same as commands_base::setIdle except that it tries twice.
/// If the device is streaming at a high rate and overrunning the serial buffer,
/// the reply from the first setIdle command may be lost. On the second attempt,
/// the device will be idle due to the first try and the ACK should come through.
///
///@param device
///
///@returns CmdResult
///
TypedResult<commands_base::SetIdle> setIdle(C::mip_interface& device)
{
    commands_base::SetIdle cmd;
    auto result = runCommand(device, cmd, 500);

    // If the previous command timed out, the response may have been dropped.
    // Try again in case the device actually did accept the command.
    if(result == CmdResult::STATUS_TIMEDOUT)
        result = runCommand(device, cmd, 500);

    return result;
}

} // namespace mip
