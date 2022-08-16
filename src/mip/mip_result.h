#pragma once

#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
///@brief Represents the status of a MIP command.
///
/// Values that start with MIP_STATUS_ are status codes from this library.
/// Values that start with MIP_(N)ACK represent replies from the device.
///
enum mip_cmd_result
{
    MIP_STATUS_ERROR     = -6,  ///< Command could not be executed (error sending/receiving)
    MIP_STATUS_CANCELLED = -5,  ///< Command was canceled in software.
    MIP_STATUS_TIMEDOUT  = -4,  ///< Reply was not received before timeout expired.
    MIP_STATUS_WAITING   = -3,  ///< Waiting for command reply (timeout timer has started).
    MIP_STATUS_PENDING   = -2,  ///< Command has been queued but the I/O update hasn't run yet.
    MIP_STATUS_NONE      = -1,  ///< Command has been initialized but not queued yet.

    MIP_ACK_OK                = 0x00,  ///< Command completed successfully.
    MIP_NACK_COMMAND_UNKNOWN  = 0x01,  ///< Command not supported.
    MIP_NACK_INVALID_CHECKSUM = 0x02,  ///< Reserved.
    MIP_NACK_INVALID_PARAM    = 0x03,  ///< A parameter was not a supported value.
    MIP_NACK_COMMAND_FAILED   = 0x04,  ///< The device could not complete the command.
    MIP_NACK_COMMAND_TIMEOUT  = 0x05,  ///< Internal device timeout.
};

const char* mip_cmd_result_to_string(enum mip_cmd_result result);

bool mip_cmd_result_is_finished(enum mip_cmd_result result);

bool mip_cmd_result_is_reply(enum mip_cmd_result result);
bool mip_cmd_result_is_status(enum mip_cmd_result result);

bool mip_cmd_result_is_ack(enum mip_cmd_result result);

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
        NACK_COMMAND_UNKNOWN  = C::MIP_NACK_COMMAND_UNKNOWN,
        NACK_INVALID_CHECKSUM = C::MIP_NACK_INVALID_CHECKSUM,
        NACK_INVALID_PARAM    = C::MIP_NACK_INVALID_PARAM,
        NACK_COMMAND_FAILED   = C::MIP_NACK_COMMAND_FAILED,
        NACK_COMMAND_TIMEOUT  = C::MIP_NACK_COMMAND_TIMEOUT,
    };

    C::mip_cmd_result value = C::MIP_STATUS_NONE;

    CmdResult() : value(C::MIP_ACK_OK) {}
    CmdResult(C::mip_cmd_result result) : value(result) {}

    CmdResult& operator=(const CmdResult& other) = default;
    CmdResult& operator=(C::mip_cmd_result other) { value = other; return *this; }

    // operator bool() const { return value == C::MIP_ACK_OK; }
    operator const void*() const { return isAck() ? this : nullptr; }
    bool operator!() const { return !isAck(); }
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
    bool isAck() const { return C::mip_cmd_result_is_ack(value); }
};

// using Ack = C::mip_ack;

} // namespace mip
#endif // __cplusplus
