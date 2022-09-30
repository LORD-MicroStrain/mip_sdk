
#include "mip_result.h"

#ifdef __cplusplus
namespace mip {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@brief Converts the command result to a string for debugging.
///
///@param result Any value.
///
///@return A constant string.
///
const char* mip_cmd_result_to_string(enum mip_cmd_result result)
{
    switch( result )
    {
        // Status codes
    case MIP_STATUS_ERROR:          return "Error";
    case MIP_STATUS_CANCELLED:      return "Canceled";
    case MIP_STATUS_TIMEDOUT:       return "Timed out";
    case MIP_STATUS_WAITING:        return "Waiting";
    case MIP_STATUS_PENDING:        return "Pending";
    case MIP_STATUS_NONE:           return "None";
    // Device replies
    case MIP_ACK_OK:                return "Ok";
    case MIP_NACK_COMMAND_UNKNOWN:  return "Unknown Command";
    case MIP_NACK_INVALID_CHECKSUM: return "Invalid Checksum";
    case MIP_NACK_INVALID_PARAM:    return "Invalid Parameter";
    case MIP_NACK_COMMAND_FAILED:   return "Command Failed";
    case MIP_NACK_COMMAND_TIMEOUT:  return "Device Error";

    default:                        return "<Unknown>";
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the command has completed, timed out, been cancelled, or
///       otherwise is no longer waiting for a response.
///
bool mip_cmd_result_is_finished(enum mip_cmd_result status)
{
    return (status >= 0) || (status <= MIP_STATUS_TIMEDOUT);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the result is a reply from the device (i.e. mip_ack).
///
bool mip_cmd_result_is_reply(enum mip_cmd_result result)
{
    return result >= 0;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the result code was generated by this lib (i.e. mip_cmd_status).
///
bool mip_cmd_result_is_status(enum mip_cmd_result result)
{
    return result < 0;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the result is an ack (successful response from the device)
///
bool mip_cmd_result_is_ack(enum mip_cmd_result result)
{
    return result == MIP_ACK_OK;
}

#ifdef __cplusplus
} // extern "C"
} // namespace mip
#endif // __cplusplus
