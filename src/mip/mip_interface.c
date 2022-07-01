
#include "mip_interface.h"

#include "mip_field.h"

#include "definitions/descriptors.h"

#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
///@brief Wrapper around MipInterface_receivePacket for use with MipParser.
///
///@param device    Void pointer to the device. Must be a MipInterfaceState pointer.
///@param packet    MIP Packet from the parser.
///@param timestamp Timestamp of the packet.
///
///@returns True
///
bool MipInterface_parseCallback(void* device, const struct MipPacket* packet, Timestamp timestamp)
{
    MipInterface_receivePacket(device, packet, timestamp);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the MipInterface components.
///
///@param device
///
///@param parseBuffer
///       A working buffer for the MIP parser. See MipParser_init().
///@param parseBufferSize
///       Size of the parsing buffer. Must be at least MIP_PACKET_LENGTH_MAX.
///@param parseTimeout
///       Maximum length of time to wait for the end of a MIP packet. See MipParser_init().
///@param baseReplyTimeout
///       Minimum time for all commands. See MipCmdQueue_init().
///
void MipInterface_init(struct MipInterfaceState* device, uint8_t* parseBuffer, size_t parseBufferSize, Timeout parseTimeout, Timeout baseReplyTimeout)
{
    MipParser_init(&device->parser, parseBuffer, parseBufferSize, &MipInterface_parseCallback, device, parseTimeout);

    device->maxPacketsPerPoll = MIPPARSER_UNLIMITED_PACKETS;

    MipCmdQueue_init(&device->queue, baseReplyTimeout);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the maximum number of packets to parser per poll call.
///
unsigned int MipInterface_maxPacketsPerPoll(const struct MipInterfaceState* device)
{
    return device->maxPacketsPerPoll;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets a limit on the number of packets which can be processed in one
///       call to the MipInterface_receiveBytes() function.
///
/// Use this when receiving data in bursts to smooth out the processing
/// load over time.
///
///@note Make sure the parsing buffer is large enough to hold the
///      data in between receive calls.
///
///@note Make sure MipInterface_userPoll() executes the parser even when no new
///      input data is available so that unparsed data is pushed through.
///
///@param device
///
///@param maxPackets
///       Maximum number of packets to parse at once.
///
void MipInterface_setMaxPacketsPerPoll(struct MipInterfaceState* device, unsigned int maxPackets)
{
    device->maxPacketsPerPoll = maxPackets;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Polls the port for new data. Called repeatedly while waiting for
///       acknowledgements to pending commands.
///
///@param device The MipInterfaceState object.
///
///@returns true if operation should continue, or false if the device cannot be
///         polled (e.g. if the serial port is not open)
///
/// Data from the port should be read and pushed into
/// MipInterface_receiveBytes() for parsing. On systems where it makes
/// sense, this is a good place to call sleep or enter a low-power state until
/// data arrives at the port. Typically this function will wait at most a few
/// milliseconds before returning.
///
/// This function is called in a loop, so returning true even when no bytes
/// have been received is always safe.
///
bool MipInterface_poll(struct MipInterfaceState* device)
{
    return MipInterface_userPoll(device);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Sends data to the port (i.e. from this library to the physical device).
///
///@param device The MipInterfaceState object.
///@param data   Data to be sent.
///@param length Length of data.
///
///@returns True if the data was sent successfully, false if some or all data
///         could not be sent.
///
/// This is called whenever bytes must be sent to the physical device.
///
bool MipInterface_sendToDevice(struct MipInterfaceState* device, const uint8_t* data, size_t length)
{
    return MipInterface_userSendToDevice(device, data, length);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Receive data from the port (i.e. the physical device) into the parser.
///
/// Call this when data has been received. In some applications, this will be
/// from the MipInterface_userPoll() function, while in others a dedicated
/// thread will manage receiving data.
///
///@param device
///
///@param data
///       Input data buffer. May be NULL if length == 0.
///@param length
///       Length of the input buffer. Must be 0 if data is NULL.
///@param timestamp
///       Time of the received data.
///
///@returns The amount of data which couldn't be processed due to the limit on
///         number of packets per parse call. Normally the result is 0.
///
RemainingCount MipInterface_receiveBytes(struct MipInterfaceState* device, const uint8_t* data, size_t length, Timestamp timestamp)
{
    return MipParser_parse(&device->parser, data, length, timestamp, device->maxPacketsPerPoll);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Process more packets from the internal buffer.
///
/// This is an alternative to MipPinterface_receiveBytes() for the case when
/// no new input data is available and maxPackets is nonzero. The timestamp is
/// reused from the last call to receiveBytes.
///
/// This function obeys the maxPacketsPerPoll setting.
///
///@note Calling this function when maxPacketsPerPoll is zero is unnecessary
///      and has no effect.
///
void MipInterface_processUnparsedPackets(struct MipInterfaceState* device)
{
    MipParser_parse(&device->parser, NULL, 0, device->maxPacketsPerPoll, MipParser_lastPacketTimestamp(&device->parser));
}

////////////////////////////////////////////////////////////////////////////////
///@brief Processes a pre-parsed packet for command replies and data.
///
///@param device
///
///@param packet
///       The received MIP packet.
///@param timestamp
///       Timestamp of the received MIP packet.
///
void MipInterface_receivePacket(struct MipInterfaceState* device, const struct MipPacket* packet, Timestamp timestamp)
{
    MipCmdQueue_processPacket(&device->queue, packet, timestamp);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the MIP parser for the device.
///
struct MipParsingState* MipInterface_parser(struct MipInterfaceState* device)
{
    return &device->parser;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the commmand queue for the device.
///
struct MipCmdQueue* MipInterface_cmdQueue(struct MipInterfaceState* device)
{
    return &device->queue;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Blocks until the pending command completes or times out.
///
///@param device
///@param cmd
///
///@returns The final status of the command.
///
MipCmdResult MipInterface_waitForReply(struct MipInterfaceState* device, const struct MipPendingCmd* cmd)
{
    MipCmdResult status;
    while( !MipCmdResult_isFinished(status = MipPendingCmd_status(cmd)) )
    {
        if( !MipInterface_poll(device) )
            return MIP_STATUS_ERROR;
    }
    return status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Runs a command using a pre-serialized payload.
///
///@param device
///@param descriptorSet
///       Command descriptor set.
///@param cmdDescriptor
///       Command field descriptor.
///@param cmdData
///       Optional payload data. May be NULL if cmdLength == 0.
///@param cmdLength
///       Length of the command payload (parameters).
///
///@return MipCmdResult, any value from MipAck or MipCmdStatus.
///        MIP_ACK_OK - Command completed successfully.
///        MIP_NACK_* - Device rejected the command.
///        MIP_STATUS_* - An error occured (e.g. timeout).
///
MipCmdResult MipInterface_runCommand(struct MipInterfaceState* device, uint8_t descriptorSet, uint8_t cmdDescriptor, const uint8_t* cmdData, uint8_t cmdLength)
{
    return MipInterface_runCommandWithResponse(device, descriptorSet, cmdDescriptor, cmdData, cmdLength, MIP_INVALID_FIELD_DESCRIPTOR, NULL, NULL);
}

////////////////////////////////////////////////////////////////////////////////
///@copydoc MipInterface_runCommand
///
///@param responseDescriptor
///       Descriptor of the response data. May be MIP_INVALID_FIELD_DESCRIPTOR
///       if no response is expected.
///
MipCmdResult MipInterface_runCommandWithResponse(struct MipInterfaceState* device,
    uint8_t descriptorSet, uint8_t cmdDescriptor, const uint8_t* cmdData, uint8_t cmdLength,
    uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t* responseLength_inout)
{
    assert(responseLength_inout != NULL);

    uint8_t buffer[MIP_PACKET_LENGTH_MAX];

    struct MipPacket packet;
    MipPacket_create(&packet, buffer, sizeof(buffer), descriptorSet);
    MipPacket_addField(&packet, cmdDescriptor, cmdData, cmdLength);
    MipPacket_finalize(&packet);

    struct MipPendingCmd cmd;
    MipPendingCmd_initWithResponse(&cmd, descriptorSet, cmdDescriptor, responseDescriptor, responseBuffer, *responseLength_inout);

    MipCmdResult result = MipInterface_runCommandPacket(device, &packet, &cmd);

    *responseLength_inout = MipPendingCmd_responseLength(&cmd);

    return result;
}

MipCmdResult MipInterface_runCommandPacket(struct MipInterfaceState* device, const struct MipPacket* packet, struct MipPendingCmd* cmd)
{
    MipCmdQueue_enqueue(MipInterface_cmdQueue(device), cmd);

    if( !MipInterface_sendToDevice(device, MipPacket_pointer(packet), MipPacket_totalLength(packet)) )
    {
        MipCmdQueue_dequeue(MipInterface_cmdQueue(device), cmd);
        return MIP_STATUS_ERROR;
    }

    return MipInterface_waitForReply(device, cmd);
}
