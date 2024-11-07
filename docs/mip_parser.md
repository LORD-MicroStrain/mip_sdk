Mip Parser {#parsing_packets}
==========

The MIP Parser takes in bytes from the device connection or recorded binary
file and extracts valid packets. Data is input to the parser and
packets are parsed out one at a time and sent to a callback function.

![](mip_parser.svg)

Parsing Data  {#parsing_data}
------------

Data is supplied by calling mip_parser_parse() / mip::Parser::parse() with
a buffer and length. Along with the data, the user must provide a timestamp.
The timestamp serves two purposes: to provide a time of reception indicator
and to allow the parser to time out waiting for more data.

The parser will scan the supplied buffer for packets, calling the packet
callback for each valid packet. This continues until the entire buffer has
been processed. Upon return, the entire buffer has been consumed.

If a valid packet gets cut off at the end of the buffer (e.g. because it
hasn't been received yet), the parser will store the partial packet internally
until more data is received. If sufficient data is not received within the
set timeout, the first byte of the potential packet is discarded and parsing
continues.

Performance
-----------

The parser is most efficient when called with large chunks of data. This is
because each call to the parser (or any function) has overhead. When reading
from a high-speed source such as a file, we recommend parsing chunks of data
of at least 512 bytes, ideally 1024 bytes or more. Little improvement is
attained beyond 8192 bytes. Performance drops off below 128-byte chunks.
This data seems to hold for both high-power desktop systems (e.g. Intel
i7-1370) and also embedded systems such as the STM32F767 at 200 MHz.
For low-speed streams performance isn't critical. In any case, the buffer
should be big enough to hold all data received in between parse calls, up to
a reasonable maximum size for your application.


Packet Timeouts  {#packet_timeouts}
---------------

In some cases it's possible for a packet to be corrupted during
transmission or reception (e.g. EMI while in transit on the wire, serial
baud rate too low, etc). If the payload length byte is corrupted, it may
falsely indicate that the packet is longer than what was sent. Without a
timeout, the parser would wait until this extra data (potentially up to 255
bytes) was received before checking and realizing that the checksum failed.
Any following packets would be delayed, possibly causing additional commands
to time out and make the device appear temporarily unresponsive. Setting a
reasonable timeout ensures that the bad packet is rejected more quickly.
The timeout should be set so that a MIP packet of the largest possible
size (261 bytes) can be transferred well within the transmission time plus
any additional processing delays in the application or operating system.
As an example, for a 115200 baud serial link a timeout of 30 ms would be
about right. You can use the mip_timeout_from_baudrate() function to
compute an appropriate timeout.

See ["microstrain_embedded_timestamp (C)"](@ref microstrain::C::microstrain_embedded_timestamp) or microstrain::EmbeddedTimestamp (C++)

The Packet Parsing Process  {#parsing_process}
--------------------------

Packets are parsed from the input buffer one at a time in the parse
function.

If a packet was previously started but not completed previously (due to
requiring more data) then the timeout is checked. If too much time has
passed, the packet is discarded and the parsing state reset. This check is
only performed once per parse call because that is the only point where the
timestamp changes.

![](parse_function.svg)

The current status is held by the `expected_length` variable, which tracks
how many bytes are expected to be in the current packet. The parse function
enters a loop, checking if there is enough data to complete the next parsing
step.

![](parse_one_packet.svg)

`expected_length` starts out as 1 when the parser is searching for the start
of a packet. Once a potential start byte (`SYNC1`) is found, the packet's
start time is initialized to the current timestamp and `expected_length` is
bumped up to the size of a mip packet header (4 bytes).

When the expected length is 4 bytes, the header's SYNC2 byte is checked for
validity and the payload length field is read. `expected_length` is set to
the full packet size (computed as the packet header and checksum size plus
the payload size).

Finally, when `expected_length` is neither of the above two conditions, it
means that the entire packet has been received. Note that other values less
than 6 (the size of an empty packet) are not possible. At this point, the
data is copied out from the ring buffer to a linear buffer for processing.
The checksum is verified, and if it passes, the entire packet is dropped
from the ring buffer and the callback function is invoked.

If any of the checks in the above steps fails, such as a wrong SYNC2 byte,
a single byte is dropped from the ring buffer and the loop is continued.
Only a single byte can be dropped, because rogue SYNC1 bytes or truncated
packets may hide real mip packets in what would have been their payload.
