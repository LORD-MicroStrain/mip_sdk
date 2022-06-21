#pragma once

#include <stdbool.h>

#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Status of a pending MIP command.
///
/// These values do not overlap with MipAck.
///
enum MipCmdStatus
{
    MIP_STATUS_NONE      = -1,  ///< Command has been initialized but not queued yet.
    MIP_STATUS_PENDING   = -2,  ///< Command has been queued.
    MIP_STATUS_WAITING   = -3,  ///< Waiting for command reply (timer started).
    MIP_STATUS_TIMEDOUT  = -5,  ///< Reply not received before timeout expired.
    MIP_STATUS_CANCELLED = -6,  ///< Command was canceled via mscl.
    MIP_STATUS_ERROR     = -7,  ///< Command could not be executed (mscl error)
};
const char* MipCmdStatus_toString(enum MipCmdStatus status);

////////////////////////////////////////////////////////////////////////////////
///@brief MIP ack/nack reply codes sent by the device in response to a command.
///
enum MipAck
{
    MIP_ACK_OK                = 0x00,  ///< Command completed successfully.
    MIP_NACK_UNKNOWN_CMD      = 0x01,  ///< Command not supported.
    MIP_NACK_INVALID_CHECKSUM = 0x02,
    MIP_NACK_INVALID_PARAM    = 0x03,  ///< A parameter was not a valid value.
    MIP_NACK_COMMAND_FAILED   = 0x04,  ///< The device could not complete the command.
    MIP_NACK_COMMAND_TIMEOUT  = 0x05,  ///< No response from the device.
};

const char* MipAck_toString(enum MipAck ack);

////////////////////////////////////////////////////////////////////////////////
///@brief Represents the result of executing a MIP command.
///
/// This can be any of the the MipCmdStatus or MipAck enum values.
///
typedef int MipCmdResult;

const char* MipCmdResult_toString(MipCmdResult result);

bool MipCmdResult_isFinished(MipCmdResult result);


#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
