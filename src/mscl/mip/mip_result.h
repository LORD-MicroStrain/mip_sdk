#pragma once

#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Status of a pending MIP command.
///
/// These values do not overlap with MipAck.
///
enum mip_cmd_status
{
    MIP_STATUS_NONE      = -1,  ///< Command has been initialized but not queued yet.
    MIP_STATUS_PENDING   = -2,  ///< Command has been queued.
    MIP_STATUS_WAITING   = -3,  ///< Waiting for command reply (timer started).
    MIP_STATUS_TIMEDOUT  = -4,  ///< Reply not received before timeout expired.
    MIP_STATUS_CANCELLED = -5,  ///< Command was canceled via mscl.
    MIP_STATUS_ERROR     = -6,  ///< Command could not be executed (mscl error)
};
const char* mip_cmd_status_to_string(enum mip_cmd_status status);

////////////////////////////////////////////////////////////////////////////////
///@brief MIP ack/nack reply codes sent by the device in response to a command.
///
enum mip_ack
{
    MIP_ACK_OK                = 0x00,  ///< Command completed successfully.
    MIP_NACK_UNKNOWN_CMD      = 0x01,  ///< Command not supported.
    MIP_NACK_INVALID_CHECKSUM = 0x02,
    MIP_NACK_INVALID_PARAM    = 0x03,  ///< A parameter was not a valid value.
    MIP_NACK_COMMAND_FAILED   = 0x04,  ///< The device could not complete the command.
    MIP_NACK_COMMAND_TIMEOUT  = 0x05,  ///< No response from the device.
};

const char* mip_ack_to_string(enum mip_ack ack);

////////////////////////////////////////////////////////////////////////////////
///@brief Represents the result of executing a MIP command.
///
/// This can be any of the the MipCmdStatus or MipAck enum values.
///
typedef int mip_cmd_result;

const char* mip_cmd_result_to_string(mip_cmd_result result);

bool mip_cmd_result_is_finished(mip_cmd_result result);

bool mip_cmd_result_is_reply(mip_cmd_result result);
bool mip_cmd_result_is_status(mip_cmd_result result);


#ifdef __cplusplus
} // extern "C"
} // namespace C

struct CmdResult
{
    enum
    {
        STATUS_ERROR          = C::MIP_STATUS_ERROR,
        STATUS_CANCELLED      = C::MIP_STATUS_CANCELLED,
        STATUS_TIMEDOUT       = C::MIP_STATUS_TIMEDOUT,
        STATUS_WAITING        = C::MIP_STATUS_WAITING,
        STATUS_PENDING        = C::MIP_STATUS_PENDING,
        STATUS_NONE           = C::MIP_STATUS_NONE,
        ACK_OK                = C::MIP_ACK_OK,
        NACK_UNKNOWN_CMD      = C::MIP_NACK_UNKNOWN_CMD,
        NACK_INVALID_CHECKSUM = C::MIP_NACK_INVALID_CHECKSUM,
        NACK_INVALID_PARAM    = C::MIP_NACK_INVALID_PARAM,
        NACK_COMMAND_FAILED   = C::MIP_NACK_COMMAND_FAILED,
        NACK_COMMAND_TIMEOUT  = C::MIP_NACK_COMMAND_TIMEOUT,
    };

    C::mip_cmd_result value = STATUS_NONE;

    CmdResult() : value(C::MIP_ACK_OK) {}
    CmdResult(C::mip_cmd_result result) : value(result) {}
    CmdResult(C::mip_ack ack)           : value(ack)    {}
    CmdResult(C::mip_cmd_status status) : value(status) {}

    CmdResult& operator=(const CmdResult& other) = default;
    CmdResult& operator=(C::mip_cmd_result other) { value = other; return *this; }

    // operator bool() const { return value == C::MIP_ACK_OK; }
    operator const void*() const { return value == C::MIP_ACK_OK ? this : nullptr; }
    bool operator!() const { return value == C::MIP_ACK_OK; }
    operator C::mip_cmd_result&() { return value; }
    operator C::mip_cmd_result() const { return value; }

    bool operator==(CmdResult other) const { return value == other.value; }
    bool operator!=(CmdResult other) const { return value != other.value; }

    bool operator==(C::mip_cmd_result other) const { return value == other; }
    bool operator!=(C::mip_cmd_result other) const { return value != other; }

    const char* name() const { return C::mip_cmd_result_to_string(value); }

    bool isReplyCode() const { return C::mip_cmd_result_is_reply(value); }
    bool isStatusCode() const { return C::mip_cmd_result_is_status(value); }
    bool isFinished() const { return C::mip_cmd_result_is_finished(value); }
};

// using Ack = C::mip_ack;

} // namespace mip
#endif // __cplusplus
