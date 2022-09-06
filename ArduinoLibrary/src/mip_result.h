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
///
typedef enum mip_cmd_result
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
    MIP_NACK_COMMAND_TIMEOUT  = 0x05,  ///< Internal device timeout. Use MIP_STATUS_TIMEDOUT for command timeouts.
} mip_cmd_result;

const char* mip_cmd_result_to_string(enum mip_cmd_result result);

bool mip_cmd_result_is_finished(enum mip_cmd_result result);

bool mip_cmd_result_is_reply(enum mip_cmd_result result);
bool mip_cmd_result_is_status(enum mip_cmd_result result);

bool mip_cmd_result_is_ack(enum mip_cmd_result result);

///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // extern "C"
} // namespace C

////////////////////////////////////////////////////////////////////////////////
///@brief Represents the status of a MIP command.
///
/// This is the same as the mip_cmd_result C enum, but with member functions
/// and some operator overloads.
///
/// CmdResult is convertible to bool, allowing code like the following:
///@code{.cpp}
///@endcode
///
struct CmdResult
{
    static constexpr C::mip_cmd_result STATUS_ERROR     = C::MIP_STATUS_ERROR;       ///<@copydoc mip::C::MIP_STATUS_ERROR
    static constexpr C::mip_cmd_result STATUS_CANCELLED = C::MIP_STATUS_CANCELLED;   ///<@copydoc mip::C::MIP_STATUS_CANCELLED
    static constexpr C::mip_cmd_result STATUS_TIMEDOUT  = C::MIP_STATUS_TIMEDOUT;    ///<@copydoc mip::C::MIP_STATUS_TIMEDOUT
    static constexpr C::mip_cmd_result STATUS_WAITING   = C::MIP_STATUS_WAITING;     ///<@copydoc mip::C::MIP_STATUS_WAITING
    static constexpr C::mip_cmd_result STATUS_QUEUED    = C::MIP_STATUS_PENDING;     ///<@copydoc mip::C::MIP_STATUS_PENDING
    static constexpr C::mip_cmd_result STATUS_NONE      = C::MIP_STATUS_NONE;        ///<@copydoc mip::C::MIP_STATUS_NONE

    static constexpr C::mip_cmd_result ACK_OK                = C::MIP_ACK_OK;                ///<@copydoc C::MIP_ACK_OK
    static constexpr C::mip_cmd_result NACK_COMMAND_UNKNOWN  = C::MIP_NACK_COMMAND_UNKNOWN;  ///<@copydoc C::MIP_NACK_COMMAND_UNKNOWN
    static constexpr C::mip_cmd_result NACK_INVALID_CHECKSUM = C::MIP_NACK_INVALID_CHECKSUM; ///<@copydoc C::MIP_NACK_INVALID_CHECKSUM
    static constexpr C::mip_cmd_result NACK_INVALID_PARAM    = C::MIP_NACK_INVALID_PARAM;    ///<@copydoc C::MIP_NACK_INVALID_PARAM
    static constexpr C::mip_cmd_result NACK_COMMAND_FAILED   = C::MIP_NACK_COMMAND_FAILED;   ///<@copydoc C::MIP_NACK_COMMAND_FAILED
    static constexpr C::mip_cmd_result NACK_COMMAND_TIMEOUT  = C::MIP_NACK_COMMAND_TIMEOUT;  ///<@copydoc C::MIP_NACK_COMMAND_TIMEOUT

#ifndef _WIN32 // Avoid name conflict with windows.h
    static constexpr C::mip_cmd_result STATUS_PENDING = STATUS_QUEUED;
#endif

    C::mip_cmd_result value = C::MIP_STATUS_NONE;

    CmdResult() : value(C::MIP_ACK_OK) {}
    CmdResult(C::mip_cmd_result result) : value(result) {}

    CmdResult& operator=(const CmdResult& other) = default;
    CmdResult& operator=(C::mip_cmd_result other) { value = other; return *this; }

    static CmdResult fromAckNack(uint8_t code) { return CmdResult(static_cast<C::mip_cmd_result>(code)); }

    operator const void*() const { return isAck() ? this : nullptr; }
    bool operator!() const { return !isAck(); }

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

///@}
////////////////////////////////////////////////////////////////////////////////

} // namespace mip
#endif // __cplusplus
