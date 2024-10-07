Timestamps and Timeouts  {#timestamps}
=======================

Timestamp Type  {#timestamp_type}
--------------

Timestamps ([mip_timestamp](@ref mip::C::mip_timestamp) / mip::Timestamp) represent the local time when data was received or a packet was parsed. These timestamps
are used to implement command timeouts and provide the user with an approximate timestamp of received data. It is not intended to be
a precise timestamp or used for synchronization, and it generally cannot be used instead of the timestamps from the connected MIP device.
In particular, if you limit the maximum number of packets processed per `update` call, the timestamp of some packets may be delayed.

Because different applications may keep track of time differently (especially on embedded platforms), it is up to the user to provide
the current time whenever data is received from the device. On a PC, this might come from the posix `clock()` function or from the
`std::chrono` library. On ARM Cortex-M systems, it is often derived from the Systick timer. It should be a monotonically increasing value;
jumps backwards in time (other than due to wraparound) will cause problems.

By default, timestamps are `typedef`'d to `uint64_t`. Typically timestamps are in milliseconds. Embedded systems may wish to use
`uint32_t` or even `uint16_t` instead. The value is allowed to wrap around as long as the time between wraparounds is longer than
twice the longest timeout needed. If higher precision is needed or wraparound can't be tolerated by your application, define it to
`uint64_t`. It must be a standard unsigned integer type.

Command Timeouts
----------------

Timeouts for commands are broken down into two parts.
* A "base reply timeout" applies to all commands. This is useful to compensate for communication latency, such as over a TCP socket.
* "Additional time" which applies per command, because some commands may take longer to complete.

Currently, only the C++ api offers a way to set the additional time parameter, and only when using the `runCommand` function taking
the command structure and the `additionalTime` parameter.

The `mip_timeout` / `mip::Timeout` typedef is an alias to the timestamp type.
