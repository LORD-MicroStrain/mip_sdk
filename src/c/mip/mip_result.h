#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Represents the status of a MIP command.
///
/// Values that start with MIP_STATUS are status codes from this library.
/// Values that start with MIP_(N)ACK represent replies from the device.
/// Values at or below MIP_STATUS_USER_START (negative values) are reserved for
/// status codes from user code.
///
/// See @ref command_results
///
typedef enum mip_cmd_result
{
    MIP_STATUS_USER_START = -10, ///< Values defined by user code must be less than or equal to this value.

    // Status codes < 0
    MIP_STATUS_ERROR     = -6,  ///< Command could not be executed (error sending/receiving)
    MIP_STATUS_CANCELLED = -5,  ///< Command was canceled in software.
    MIP_STATUS_TIMEDOUT  = -4,  ///< Reply was not received before timeout expired.
    MIP_STATUS_WAITING   = -3,  ///< Waiting for command reply (timeout timer has started).
    MIP_STATUS_PENDING   = -2,  ///< Command has been queued but the I/O update hasn't run yet.
    MIP_STATUS_NONE      = -1,  ///< Command has been initialized but not queued yet.

    // Device replies >= 0
    MIP_ACK_OK                = 0x00,  ///< Command completed successfully.
    MIP_NACK_COMMAND_UNKNOWN  = 0x01,  ///< Command not supported.
    MIP_NACK_INVALID_CHECKSUM = 0x02,  ///< Reserved.
    MIP_NACK_INVALID_PARAM    = 0x03,  ///< A parameter was not a supported value.
    MIP_NACK_COMMAND_FAILED   = 0x04,  ///< The device could not complete the command.
    MIP_NACK_COMMAND_TIMEOUT  = 0x05,  ///< Internal device timeout. Use MIP_STATUS_TIMEDOUT for command timeouts.
} mip_cmd_result;

const char *mip_cmd_result_to_string(enum mip_cmd_result result);

bool mip_cmd_result_is_finished(enum mip_cmd_result result);

bool mip_cmd_result_is_reply(enum mip_cmd_result result);
bool mip_cmd_result_is_status(enum mip_cmd_result result);
bool mip_cmd_result_is_user(enum mip_cmd_result result);

bool mip_cmd_result_is_ack(enum mip_cmd_result result);

///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus
