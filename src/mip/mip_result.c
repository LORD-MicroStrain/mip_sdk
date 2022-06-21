
#include "mip_result.h"

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Converts the status code to a string for debugging.
///
///@param status
///
///@returns A constant string.
///
const char* MipCmdStatus_toString(enum MipCmdStatus status)
{
    switch(status)
    {
    case MIP_STATUS_NONE:      return "MIP_STATUS_NONE";
    case MIP_STATUS_PENDING:   return "MIP_STATUS_PENDING";
    case MIP_STATUS_WAITING:   return "MIP_STATUS_WAITING";
    // case MIP_STATUS_COMPLETED: return "MIP_STATUS_COMPLETED";
    case MIP_STATUS_TIMEDOUT:  return "MIP_STATUS_TIMEDOUT";
    case MIP_STATUS_CANCELLED: return "MIP_STATUS_CANCELLED";
    case MIP_STATUS_ERROR:     return "MIP_STATUS_ERROR";
    }
    return "MIP_STATUS_INVALID";
}

////////////////////////////////////////////////////////////////////////////////
///@brief Converts the ack/nack code to a string.
///
///@param ack
///@returns A constant string.
///
const char* MipAck_toString(enum MipAck ack)
{
    switch(ack)
    {
    case MIP_ACK_OK:                return "Ok";
    case MIP_NACK_UNKNOWN_CMD:      return "Unknown Command";
    case MIP_NACK_INVALID_CHECKSUM: return "Invalid Checksum";
    case MIP_NACK_INVALID_PARAM:    return "Invalid Parameter";
    case MIP_NACK_COMMAND_FAILED:   return "Command Failed";
    case MIP_NACK_COMMAND_TIMEOUT:  return "Timed out";
    }
    return "Invalid";
}

////////////////////////////////////////////////////////////////////////////////
///@brief Converts the command result to a string for debugging.
///
///@param result Any value, generally from the MipCmdStatus or MipAck enum.
///@return A constant string.
///
const char* MipCmdResult_toString(MipCmdResult result)
{
    if( result >= 0 )
        return MipAck_toString((enum MipAck)result);
    else
        return MipCmdStatus_toString((enum MipCmdStatus)result);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the command has completed, timed out, been cancelled, or
///       otherwise is no longer waiting for a response.
///
bool MipCmdResult_isFinished(enum MipCmdStatus status)
{
    return (status >= 0) || (status <= MIP_STATUS_TIMEDOUT);
}

#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
