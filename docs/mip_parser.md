Mip Parser {#parsing_packets}
==========

The MIP Parser takes in bytes from the device connection or recorded binary
file and extracts the packet data. Data is input to the ring buffer and
packets are parsed out one at a time and sent to a callback function.

The parser uses a ring buffer to store data temporarily between reception
and parsing. This helps even out processor workload on embedded systems
when data arrives in large bursts.

![](mip_parser.svg)

Parsing Data  {#parsing_data}
------------

Data is supplied by calling mip_parser_parse() / mip::Parser::parse() with
a buffer and length. Along with the data, the user must provide a timestamp.
The timestamp serves two purposes: to provide a time of reception indicator
and to allow the parser to time out waiting for more data.

The parse function takes an additional parameter, `max_packets`, which
limits the number of packets parsed. This can be used to prevent a large
quantity of packets from consuming too much CPU time and denying service
to other subsystems. If the limit is reached, parsing stops and the
unparsed portion of the data remains in the ring buffer. The constant value
MIPPARSER_UNLIMITED_PACKETS disables this limit.

To continue parsing, call the parse function again. You may choose to
not supply any new data by passing NULL and a length of 0. The timestamp
should be unchanged from the previous call for highest accuracy, but it's
permissible to use the current time as well. If new data is supplied, the
new data is appended to the ring buffer and parsing resumes. The timestamp
should be the time of the new data. Previously received but unparsed packets
will be assigned the new timestamp.

The application must parse enough packets to keep up with the incoming
data stream. Failure to do so will result in the ring buffer becoming
full. If this happens, the parse function will return a negative number,
indicating the number of bytes that couldn't be copied. This will never
happen if max_packets is `MIPPARSER_UNLIMITED_PACKETS` because all
of the data will be processed as soon as it is received.

The Ring Buffer  {#ring_buffer}
---------------

The ring buffer's backing buffer is a byte array that is allocated by
the application during initialization. It must be large enough to store
the biggest burst of data seen at any one time. For example, applications
expecting to deal with lots of GNSS-related data will need a bigger buffer
because there may be a large number of satellite messages. These messages
are sent relatively infrequently but contain a lot of data. If max_packets
is `MIPPARSER_UNLIMITED_PACKETS`, then it needs only 512 bytes (enough for
one packet, rounded up to a power of 2).

In addition to passing data to the parse function, data can be written
directly to the ring buffer by obtaining a writable pointer and length
from mip_parser_get_write_ptr(). This may be more efficient by skipping a
copy operation. Call mip_parser_process_written() to tell the parser how
many bytes were written to the pointer. Note that the length returned by
`mip_parser_get_write_ptr` can frequently be less than the total
available space. An application should call it in a loop as long as there
is more data to process and the returned size is greater than 0.

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

See ["mip_timestamp (C)"](@ref mip_timestamp) or [mip::Timestamp (C++)](@ref mip::Timestamp)

The Packet Parsing Process  {#parsing_process}
--------------------------

Packets are parsed from the internal ring buffer one at a time in the parse
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
