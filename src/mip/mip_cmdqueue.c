
#include "mip_cmdqueue.h"

#include "mip_field.h"
#include "mip_packet.h"

#include <string.h>
#include <assert.h>


#define MIP_REPLY_DESC_GLOBAL_ACK_NACK 0xF1

#define MIP_INDEX_REPLY_DESCRIPTOR 0
#define MIP_INDEX_REPLY_ACK_CODE   1


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending command with no reponse data or additional time.
///
///@param cmd
///@param descriptorSet
///       Command descriptor set.
///@param fieldDescriptor
///       Command field descriptor.
///
void MipPendingCmd_init(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor)
{
    MipPendingCmd_initFull(cmd, descriptorSet, fieldDescriptor, 0x00, NULL, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with extra timeout time.
///
///@param cmd
///@param descriptorSet
///       Command descriptor set.
///@param fieldDescriptor
///       Command field descriptor.
///@param additionalTime
///       Additional time on top of the base reply timeout for this specific command.
///
void MipPendingCmd_initWithTimeout(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor, Timeout additionalTime)
{
    MipPendingCmd_initFull(cmd, descriptorSet, fieldDescriptor, 0x00, NULL, 0, additionalTime);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with expected response data.
///
///@param cmd
///@param descriptorSet
///       Command descriptor set.
///@param fieldDescriptor
///       Command field descriptor.
///@param responseDescriptor
///       Optional response data descriptor. Use 0x00 if no data is expected.
///@param responseBuffer
///       Optional buffer to hold response data, if any. If NULL, responseBufferSize must be 0.
///@param responseBufferSize
///       Optional buffer to hold response data, if any. If NULL, responseBufferSize must be 0.
///@param responseBufferSize
///       Size of the response buffer. The response will be limited to this size.
///
void MipPendingCmd_initWithResponse(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor, uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t responseBufferSize)
{
    MipPendingCmd_initFull(cmd, descriptorSet, fieldDescriptor, responseDescriptor, responseBuffer, responseBufferSize, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with all parameters.
///
///@param cmd
///@param descriptorSet
///       Command descriptor set.
///@param fieldDescriptor
///       Command field descriptor.
///@param responseDescriptor
///       Optional response data descriptor. Use 0x00 if no data is expected.
///@param responseBuffer
///       Optional buffer to hold response data, if any. If NULL, responseBufferSize must be 0.
///@param responseBufferSize
///       Size of the response buffer. The response will be limited to this size.
///@param additionalTime
///       Additional time on top of the base reply timeout for this specific command.
///
void MipPendingCmd_initFull(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor, uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime)
{
    cmd->next               = NULL;
    cmd->responseBuffer     = NULL;
    cmd->extraTimeout       = additionalTime;
    cmd->descriptorSet      = descriptorSet;
    cmd->fieldDescriptor    = fieldDescriptor;
    cmd->responseDescriptor = responseDescriptor;
    cmd->responseBuffer     = responseBuffer;
    cmd->responseBufferSize = responseBufferSize;
    // cmd->ackCode            = 0xFF; // invalid
    cmd->status             = MIP_STATUS_NONE;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the status of the pending command.
///
///@see MipCmdStatus
///
enum MipCmdStatus MipPendingCmd_status(const struct MipPendingCmd* cmd)
{
    return cmd->status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the response payload pointer.
///
/// This function may only be called after the command finishes with an ACK.
///
const uint8_t* MipPendingCmd_response(const struct MipPendingCmd* cmd)
{
    assert(MipCmdResult_isFinished(cmd->status));

    return cmd->responseBuffer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the response data.
///
/// This function may only be called after the command finishes.
/// If the command completed with a NACK, or if it timed out, the response
/// length will be zero.
///
uint8_t MipPendingCmd_responseLength(const struct MipPendingCmd* cmd)
{
    assert(MipCmdResult_isFinished(cmd->status));

    return cmd->responseLength;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Checks if the command should time out.
///
///@param cmd
///@param now Current time
///
///@returns true if the command should time out. Only possible for MIP_STATUS_WAITING.
///@returns false if the command should not time out.
///
bool MipPendingCmd_checkTimeout(const struct MipPendingCmd* cmd, Timestamp now)
{
    if( cmd->status == MIP_STATUS_WAITING )
    {
        if( (int)(now - cmd->timeoutTime) > 0 )
        {
            return true;
        }
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Initializes a command queue.
///
///@param queue
///@param baseReplyTimeout
///       The minimum timeout given to all MIP commands. Additional time may be
///       given to specific commands which take longer. This is intended to be
///       used to accommodate long communication latencies, such as when using
///       a TCP connection.
///
void MipCmdQueue_init(struct MipCmdQueue* queue, Timeout baseReplyTimeout)
{
    queue->firstPendingCmd = NULL;
    queue->baseTimeout = baseReplyTimeout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Queue a command to wait for replies.
///
///@param queue
///@param cmd Listens for replies to this command.
///
///@warning The command must not be deallocated or go out of scope while the
///         MipCmdStatus_isFinished returns false.
///
void MipCmdQueue_enqueue(struct MipCmdQueue* queue, struct MipPendingCmd* cmd)
{
    // For now only one command can be queued at a time.
    if( queue->firstPendingCmd )
    {
        cmd->status = MIP_STATUS_CANCELLED;
        return;
    }

    cmd->status = MIP_STATUS_PENDING;
    queue->firstPendingCmd = cmd;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Removes a pending command from the queue.
///
///@internal
///
///@param queue
///@param cmd
///
void MipCmdQueue_dequeue(struct MipCmdQueue* queue, struct MipPendingCmd* cmd)
{
    if( queue->firstPendingCmd == cmd )
    {
        queue->firstPendingCmd = NULL;
        cmd->status = MIP_STATUS_CANCELLED;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Iterate over a packet, checking for replies to the pending command.
///
///@internal
///
///@param pending
///       Pending command which is awaiting replies
///@param packet
///@param baseTimeout
///@param timestamp
///
///@returns The new status of the pending command (the command status field is
///         not updated). The caller should set pending->status to this value
///         after doing any additional processing requiring the pending struct.
///
static enum MipCmdStatus processFieldsForPendingCmd(struct MipPendingCmd* pending, const struct MipPacket* packet, Timeout baseTimeout, Timestamp timestamp)
{
    assert( pending->status != MIP_STATUS_NONE );         // pending->status must be set to MIP_STATUS_PENDING in MipCmdQueue_enqueue to get here.
    assert( !MipCmdResult_isFinished(pending->status) );  // Command shouldn't be finished yet - make sure the queue is processed properly.

    if( pending->status == MIP_STATUS_PENDING )
    {
        // Update the timeout to the timestamp of the timeout time.
        pending->timeoutTime = timestamp + baseTimeout + pending->extraTimeout;
        pending->status = MIP_STATUS_WAITING;
    }

    // ------+------+------+------+------+------+------+------+------+------------------------
    //  ...  | 0x02 | 0xF1 | cmd1 | nack | 0x02 | 0xF1 | cmd2 |  ack |  response field ...
    // ------+------+------+------+------+------+------+------+------+------------------------

    if( MipPacket_descriptorSet(packet) == pending->descriptorSet )
    {
        for(struct MipField field = MipField_fromPacket(packet); MipField_isValid(&field); MipField_next(&field))
        {
            // Not an ack/nack reply field, skip it.
            if( MipField_fieldDescriptor(&field) != MIP_REPLY_DESC_GLOBAL_ACK_NACK )
                continue;

            // Sanity check payload length before accessing it.
            if( MipField_payloadLength(&field) != 2 )
                continue;

            const uint8_t* const payload = MipField_payload(&field);

            const uint8_t cmdDescriptor = payload[MIP_INDEX_REPLY_DESCRIPTOR];
            const uint8_t ackCode       = payload[MIP_INDEX_REPLY_ACK_CODE];

            // Is this the right command reply?
            if( pending->fieldDescriptor != cmdDescriptor )
                continue;

            // Descriptor matches!

            uint8_t responseLength = 0;
            struct MipField responseField;

            // If the command was ACK'd, check if response data is expected.
            if( pending->responseDescriptor != 0x00 && ackCode == MIP_ACK_OK )
            {
                // Look ahead one field for response data.
                responseField = MipField_nextAfter(&field);
                if( MipField_isValid(&responseField) )
                {
                    const uint8_t responseDescriptor = MipField_fieldDescriptor(&responseField);

                    // This is a wildcard to accept any response data descriptor.
                    // Needed when the response descriptor is not known or is wrong.
                    if( pending->responseDescriptor == MIP_REPLY_DESC_GLOBAL_ACK_NACK )
                        pending->responseDescriptor = responseDescriptor;

                    // Make sure the response descriptor matches what is expected.
                    if( responseDescriptor == pending->responseDescriptor )
                    {
                        // Update the responseSize field to reflect the actual size.
                        responseLength = MipField_payloadLength(&responseField);

                        // Skip this field when iterating for next ack/nack reply.
                        field = responseField;
                    }
                }
            }

            // Limit response data size to lesser of buffer size or actual response length.
            pending->responseLength = (responseLength < pending->responseBufferSize) ? responseLength : pending->responseBufferSize;

            // Copy response data to the pending buffer (skip if responseField is invalid).
            if( pending->responseLength > 0 )
                memcpy(pending->responseBuffer, MipField_payload(&responseField), pending->responseLength);

            // pending->ackCode   = ackCode;
            pending->replyTime = timestamp;  // Completion time

            return ackCode;
        }
    }

    // No matching reply descriptors in this packet.

    // Check for timeout
    if( MipPendingCmd_checkTimeout(pending, timestamp) )
    {
        pending->responseLength = 0;
        // pending->ackCode        = MIP_NACK_COMMAND_TIMEOUT;

        // Must be last!
        return MIP_STATUS_TIMEDOUT;
    }

    return pending->status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Process an incoming packet and check for replies to pending commands.
///
/// Call this from the MipParser callback, passing the arguments directly.
///
///@param queue
///@param packet The received MIP packet. Assumed to be valid.
///@param timestamp The time the packet was received
///
void MipCmdQueue_processPacket(struct MipCmdQueue* queue, const struct MipPacket* packet, Timestamp timestamp)
{
    // Check if the packet is a command descriptor set.
    const uint8_t descriptorSet = MipPacket_descriptorSet(packet);
    if( descriptorSet >= 0x80 && descriptorSet < 0xF0 )
        return;

    if( queue->firstPendingCmd )
    {
        struct MipPendingCmd* pending = queue->firstPendingCmd;

        const MipCmdResult status = processFieldsForPendingCmd(pending, packet, queue->baseTimeout, timestamp);

        if( MipCmdResult_isFinished(status) )
        {
            queue->firstPendingCmd = queue->firstPendingCmd->next;

            // This must be done last b/c it may trigger the thread which queued the command.
            // The command could go out of scope or its attributes inspected.
            pending->status = status;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Call periodically to make sure commands time out if no packets are
///       received.
///
/// Call this during the device update if no calls to MipCmdQueue_processPacket
/// are made (e.g. because no packets were received). It is safe to call this
/// in either case.
///
///@param queue
///@param now
///
void MipCmdQueue_update(struct MipCmdQueue* queue, Timestamp now)
{
    if( queue->firstPendingCmd )
    {
        struct MipPendingCmd* pending = queue->firstPendingCmd;

        if( pending->status == MIP_STATUS_PENDING )
        {
            // Update the timeout to the timestamp of the timeout time.
            pending->timeoutTime = now + queue->baseTimeout + pending->extraTimeout;
            pending->status = MIP_STATUS_WAITING;
        }
        else if( MipPendingCmd_checkTimeout(pending, now) )
        {
            queue->firstPendingCmd = queue->firstPendingCmd->next;

            // Clear response length and mark when it timed out.
            pending->responseLength = 0;
            pending->replyTime = now;

            // This must be last!
            pending->status = MIP_STATUS_TIMEDOUT;
        }
    }
}
