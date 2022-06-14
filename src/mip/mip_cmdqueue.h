#pragma once

#include <stdint.h>

#include "../types.h"

#ifdef __cplusplus
namespace mscl{
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipCommandHandling MIP Command Handling - Functions for handling command responses.
///
///@{


////////////////////////////////////////////////////////////////////////////////
///@brief Status of a pending MIP command.
///
/// These values do not overlap with MipAck.
///
enum MipCmdStatus
{
    MIP_STATUS_NONE      = -1,  ///< Command has been initialized but not queued yet.
    MIP_STATUS_PENDING   = -2,  ///< Command has been queued.
    MIP_STATUS_WAITING   = -3,  ///< Waiting for command reply (timer started).
    MIP_STATUS_TIMEDOUT  = -5,  ///< Reply not received before timeout expired.
    MIP_STATUS_CANCELLED = -6,  ///< Command was canceled via mscl.
    MIP_STATUS_ERROR     = -7,  ///< Command could not be executed (mscl error)
};
const char* MipCmdStatus_toString(enum MipCmdStatus status);

////////////////////////////////////////////////////////////////////////////////
///@brief MIP ack/nack reply codes sent by the device in response to a command.
///
enum MipAck
{
    MIP_ACK_OK                = 0x00,  ///< Command completed successfully.
    MIP_NACK_UNKNOWN_CMD      = 0x01,  ///< Command not supported.
    MIP_NACK_INVALID_CHECKSUM = 0x02,
    MIP_NACK_INVALID_PARAM    = 0x03,  ///< A parameter was not a valid value.
    MIP_NACK_COMMAND_FAILED   = 0x04,  ///< The device could not complete the command.
    MIP_NACK_COMMAND_TIMEOUT  = 0x05,  ///< No response from the device.
};

const char* MipAck_toString(enum MipAck ack);

////////////////////////////////////////////////////////////////////////////////
///@brief Represents the result of executing a MIP command.
///
/// This can be any of the the MipCmdStatus or MipAck enum values.
///
typedef int MipCmdResult;

const char* MipCmdResult_toString(MipCmdResult result);

bool MipCmdResult_isFinished(MipCmdResult result);


////////////////////////////////////////////////////////////////////////////////
///@defgroup PendingCommand  MipPendingCmd functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a command awaiting a reply from the device.
///
struct MipPendingCmd
{
    struct MipPendingCmd*      next;                ///<@private Next command in the queue.
    uint8_t*                   responseBuffer;      ///<@private Buffer for response data if responseDescriptor != 0x00.
    union {
        Timeout                extraTimeout;        ///<@private If MIP_STATUS_PENDING:   Duration to wait for reply, excluding base timeout time from the queue object.
        Timestamp              timeoutTime;         ///<@private If MIP_STATUS_WAITING:   Timestamp after which the command will be timed out.
        Timestamp              replyTime;           ///<@private If MIP_STATUS_COMPLETED: Timestamp from the packet containing the ack/nack.
    };
    uint8_t                    descriptorSet;       ///<@private Command descriptor set.
    uint8_t                    fieldDescriptor;     ///<@private Command field descriptor.
    uint8_t                    responseDescriptor;  ///<@private Response field descriptor, or 0x00 if no response field expected.
    union {
        uint8_t                responseBufferSize;  ///<@private If status < MIP_STATUS_COMPLETED, the size of the reply data buffer.
        uint8_t                responseLength;      ///<@private If status == MIP_STATUS_COMPLETED, the length of the reply data.
    };
    volatile MipCmdResult      status;              ///<@private The current status of the command. Writing this to any MipAck value may cause deallocation.
};

void MipPendingCmd_init(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor);
void MipPendingCmd_initWithTimeout(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor, Timeout additionalTime);
void MipPendingCmd_initWithResponse(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor, uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t responseBufferSize);
void MipPendingCmd_initFull(struct MipPendingCmd* cmd, uint8_t descriptorSet, uint8_t fieldDescriptor, uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t responseSize, Timeout additionalTime);

MipCmdResult MipPendingCmd_status(const struct MipPendingCmd* cmd);

const uint8_t* MipPendingCmd_response(const struct MipPendingCmd* cmd);
uint8_t MipPendingCmd_responseLength(const struct MipPendingCmd* cmd);

bool MipPendingCmd_checkTimeout(const struct MipPendingCmd* cmd, Timestamp now);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup CommandQueue  MipCmdQueue functions
///
///@{

struct MipCmdQueue
{
    struct MipPendingCmd* firstPendingCmd;
    Timeout baseTimeout;
};

void MipCmdQueue_init(struct MipCmdQueue* queue, Timeout baseReplyTimeout);
void MipCmdQueue_enqueue(struct MipCmdQueue* queue, struct MipPendingCmd* cmd);

void MipCmdQueue_update(struct MipCmdQueue* queue, Timestamp timestamp);

struct MipPacket;

void MipCmdQueue_processPacket(struct MipCmdQueue* queue, const struct MipPacket* packet, Timestamp timestamp);


///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace mscl
} // namespace C
} // extern "C"
#endif
