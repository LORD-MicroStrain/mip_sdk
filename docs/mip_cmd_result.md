Mip Command Results  {#command_results}
===================

All of the command functions in the MIP SDK return a Command Result. Command results are represented by an 8-bit enum
type in both C and C++:

* [mip_command_result (C)](@ref mip::C::mip_cmd_result)
* [mip::CmdResult (C++)](@ref mip::CmdResult)

Command results are divided into two categories, reply codes and status
codes. Reply codes are returned by the device directly:
* `ACK_OK` - Ok; The command was successful.
* `NACK_COMMAND_UNKNOWN` - Unknown command; the descriptor was not recognized or isn't supported.
* `NACK_INVALID_PARAM`   - Invalid parameter; One or more parameters are missing, invalid, or not supported.
* `NACK_COMMAND_FAILED`  - Command failed; The device was unable to perform the requested function.

The values of these enums match the corresponding values returned by the device in the MIP protocol. They are
non-negative 8-bit integers. Please see the documentation for your device for more details.

Status codes are set by this library and are _negative_ 8-bit integers:
* `STATUS_ERROR` - General error, such as failure to send/receive on the connection.
* `STATUS_TIMEDOUT` - Timeout; No response was heard from the device within the specified time period.
* Other statuses are used to track commands in progress
* `STATUS_USER` - Application-specific codes can also be set. They must be this value or lower (more negative).

You can use [mip_cmd_result_is_reply()](@ref mip::C::mip_cmd_result_is_reply) / mip::CmdResult::isReplyCode() and
[mip_cmd_result_is_status()](@ref mip::C::mip_cmd_result_is_status) / mip::CmdResult::isStatusCode() to distinguish
between the two categories.

In C++, CmdResult is implicitly convertible to bool. `ACK_OK` converts to true while everything else converts to false.
This allows compact code like
~~~~~~~~{.cpp}
if( !mip::commands_base::resume(device) )  // resume returns a CmdResult
    std::fprintf(stderr, "Failed to resume the sensor\n");
~~~~~~~~

For debugging, the name of command results is available via
[mip_cmd_result_to_string()](@ref mip::C::mip_cmd_result_to_string) / mip::CmdResult::name

In C++, CmdResult defaults to the initial state `CmdResult::STATUS_NONE`.
