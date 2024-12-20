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

The parse function should be called regularly, even if no new data has been
received (pass 0 for the input length). Otherwise, packets will not be able
to time out until new data arrives. When parsing a file, it is recommended
to call mip_parser_flush() when the end of file is reached to forcibly
time out any bad packets near the end.

Mip packets may be interspersed with other protocols. As long as the mip
packets are not fragmented this parser will reject other data and still
process the valid packets. Note that checksums are not foolproof however,
and it is theoretically possible that random data may appear to be a valid
mip packet. For this to occur it would have to contain the sync bytes 0x75,
0x65, and a valid checksum which could occur with a 1/65536 chance. The
probability of this happening is extremely low in most cases. (For a run of
6 bytes, the minimum packet size with no payload, the probability is
1/4,294,967,296 since there are 4 bytes which would have to match exactly.)
Further still, for an application to process such a packet it would also
have to have a recognized descriptor set, appropriate field lengths, and
recognized field descriptors.

#### Direct parsing

Data may be read directly into the parser's internal buffer. This may be
useful for memory-constrained systems where buffer space is limited.
E.g. a UART "byte received" IRQ routine could write directly to the
parser buffer one byte at a time.

To do this, call mip_parser_get_write_ptr() to obtain a writable pointer and
amount of available space. Then call mip_parser_parse() with a NULL input buffer
and input length equal to the number of bytes written. At least one byte of
free space will always be available, assuming the parse function has been called
between writes.

Avoid using this feature when your data is already in a contiguous buffer
since just passing it to the parse function will be significantly faster.
Also avoid mixing this method with parsing data from a buffer like normal.


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

![](mip_packet_timeout.svg)

The figure above shows a bad packet (or random data that looks like a packet)
with a length field that exceeds the number of received bytes. Within that
range, there is a valid packet that has been fully received. The timeout
allows the parser to process the inner packet without waiting for more data
to arrive.

The timeout should be set so that a MIP packet of the largest possible
size (261 bytes) can be transferred well within the transmission time plus
any additional processing delays in the application or operating system.
As an example, for a 115200 baud serial link a timeout of 30 ms would be
about right. You can use the mip_timeout_from_baudrate() function to
compute an appropriate timeout.

See ["microstrain_embedded_timestamp (C)"](@ref microstrain::C::microstrain_embedded_timestamp) or microstrain::EmbeddedTimestamp (C++)

The Packet Parsing Process  {#parsing_process}
--------------------------

The parser contains the following state:
* A timeout, used to determine how long it could take to receive one packet
* The reception timestamp of the start of the most recent packet
* An internal buffer big enough to hold a complete packet
* A counter indicating the number of valid bytes stored in the internal buffer
* Some diagnostics, if enabled by MIP_ENABLE_DIAGNOSTICS.

The parsing algorithm is centered on a quantity called `expected_packet_length`,
which indicates the number of bytes needed for the packet currently being
parsed. It also acts as the parser's state machine. The possible states are as
follows. If sufficient data is available, the state is advanced as described
here:
* 1 - No packet found yet; search for the start of the next packet (SOP).
  New expected_packet_length is 2.
* 2 - SYNC1 (SOP) byte found; check if the next byte SYNC2. New
  expected_packet_length is 4. 
* 4 - SYNC1 and SYNC2 received. Read the payload length from the buffer. Update 
  expected_packet_length to the full packet length.
* N>=6 - The full length of the packet is known; Verify the checksum and call
  the packet callback if valid. Then erase the packet data and restart parsing.

If insufficient data is available, parsing cannot continue. In this case, the
parser function must return to allow the application to fetch more data.
However, a timeout check is made first in case the current "packet" is bogus.
Before returning, all remaining data is copied from the input buffer to the
parser's internal buffer.

After a packet is processed (valid or otherwise), any remaining bytes in the
internal buffer get moved to the start of the buffer. Parsing continues until
insufficient data is available. Before returning, all remaining input data is
copied from the input buffer to the parser's internal buffer.

"Available data" means the total number of bytes in either the internal parser
buffer or the input buffer. For efficiency reasons, data is only moved to the
internal buffer when:
* A complete packet is received (before validating the checksum), or
* The parse function returns (which can only be due to lack of available data).

![](mip_parser_parse.svg)

To improve parsing efficiency, the parser leverages string functions from the C standard
library, namely memcpy() and memchr(). These functions are likely to be heavily optimized
for each platform. For example, memchr is used inside mip_find_sop(), which searches for
the start of a packet. This is faster than iterating the parser loop and dropping one byte
at a time until a sync byte is found.

Original versions of this parser used a ring buffer, which is common on embedded systems
for things like UART drivers. The ring buffer was dropped in favor of using memmove() to
shift unparsed data in-place. This reduces the number of times each byte must be
copied and also speeds up access.

The packet view passed to the callback always references the internal parser buffer. It's
safe to store a reference to this packet until further calls to parse or any other
function which may alter the parser's state.

Performance
-----------

The parser is most efficient when called with large chunks of data. This is
because each call to the parser (or any function) has overhead. When reading
from a high-speed source such as a file, we recommend parsing chunks of data
of at least 512 bytes, ideally 1024 bytes or more. Little improvement is
attained beyond 8192 bytes. Performance drops off below 128-byte chunks.

This data seems to hold for both high-power desktop systems (e.g. Intel
i7-1370 and i7-12700K) and embedded systems such as the STM32F767 at 200 MHz.
You can benchmark your system by running the TestMipPerf test program.

For low-speed streams performance isn't critical. In any case, the buffer
should be big enough to hold all data received in between parse calls, up to
a reasonable maximum size for your application.
