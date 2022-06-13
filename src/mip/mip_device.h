#pragma once

#include <stdint.h>
#include <stddef.h>

#include "mip_parser.h"
#include "mip_cmdqueue.h"

#ifdef __cplusplus
namespace mscl{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipInterface  High-level functions for controlling a MIP device.
///
///- Sending commands
///- Receiving Data
///
/// There are two ways to handle input data:
/// 1. Call MipInterface_receiveBytes (and/or MipInterface/processUnparsedPackets)
///    from MipInterface_userPoll(), or
/// 2. Run a separate thread which feeds data from the port into
///    MipInterface_receiveBytes(). MipInterface_userPoll() would then just do
///    nothing or sleep/yield.
///
/// Example of the first approach:
///@code{.c}
/// bool MipInterface_userPoll(struct MipInterfaceState* device)
/// {
///     size_t count;
///     uint8_t buffer[N];
///     if( user_readPort(buffer, sizeof(buffer), &count) == ERROR )
///         return false;  // Abort further processing if the port is closed.
///
///     MipInterface_receiveBytes(device, buffer, count, user_getTime(), MIPPARSER_UNLIMITED_PACKETS);
/// }
///@endcode
///
/// Example of the second approach:
///@code{.c}
/// bool MipInterface_userPoll(struct MipInterfaceState* device)
/// {
///     user_sleep();
///     return true;
/// }
///
/// // In another thread
/// for(;;)
/// {
///     user_waitForData();
///     MipInterface_receiveBytes(device, buffer, count, user_getTime(), MIPPARSER_UNLIMITED_PACKETS);
/// }
///@endcode
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief State of the interface for communicating with a MIP device.
///
struct MipInterfaceState
{
    struct MipParsingState parser;            ///<@private MIP Parser for incoming MIP packets.
    struct MipCmdQueue     queue;             ///<@private Queue for checking command replies.
    unsigned int           maxPacketsPerPoll; ///<@private Max number of MIP packets to parse at once.
};


void MipInterface_init(struct MipInterfaceState* device, uint8_t* parseBuffer, size_t parseBufferSize, Timeout parseTimeout, Timeout baseReplyTimeout);

void MipInterface_setMaxPacketsPerPoll(struct MipInterfaceState* device, unsigned int maxPackets);
unsigned int MipInterface_maxPacketsPerPoll(const struct MipInterfaceState* device);

RemainingCount MipInterface_receiveBytes(struct MipInterfaceState* device, const uint8_t* data, size_t length, Timestamp timestamp);
void MipInterface_processUnparsedPackets(struct MipInterfaceState* device);
bool MipInterface_poll(struct MipInterfaceState* device);

bool MipInterface_sendToDevice(struct MipInterfaceState* device, const uint8_t* data, size_t length);

bool MipInterface_parseCallback(void* device, const struct MipPacket* packet, Timestamp timestamp);
void MipInterface_receivePacket(struct MipInterfaceState* device, const struct MipPacket* packet, Timestamp timestamp);


struct MipParsingState* MipInterface_parser(struct MipInterfaceState* device);
struct MipCmdQueue*     MipInterface_cmdQueue(struct MipInterfaceState* device);


////////////////////////////////////////////////////////////////////////////////
///@defgroup UserFunctions  User-implemented callback functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@copydoc MipInterface_poll
///
///@note If a limit is placed on the max number of packets to parse at once,
///      Make sure to call MipInterface_receiveBytes(), or at least
///      MipParser_parseOnePacketFromRing() on the parser, even if no data is
///      available. Otherwise command replies and data may not get through the
///      system quickly enough.
///
///@warning Do not block indefinitely as this will stall the system beyond the
///         normal command timeout. Use a sensible timeout (i.e. 1/10th of the
///         base reply timeout) or only sleep for a minimal amount of time.
///
extern bool MipInterface_userPoll(struct MipInterfaceState* device);


////////////////////////////////////////////////////////////////////////////////
///@copydoc MipInterface_sendToDevice
///
///@note This is a good place to put logging code for debugging device
///      communications at the byte level.
///
///@note There are cases where the data will not be a MIP packet.
///
extern bool MipInterface_userSendToDevice(struct MipInterfaceState* device, const uint8_t* data, size_t length);


///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mscl
} // namespace C
} // extern "C"
#endif
