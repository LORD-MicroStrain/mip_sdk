
#pragma once

#include <stdint.h>

#include "../types.h"
#include "mip_result.h"

#ifdef __cplusplus
namespace mscl {
namespace C {
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipCommandHandling MIP Command Handling - Functions for handling command responses.
///
///@{


////////////////////////////////////////////////////////////////////////////////
///@defgroup PendingCommand  mip_pending_cmd functions
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a command awaiting a reply from the device.
///
struct mip_pending_cmd
{
    struct mip_pending_cmd*    _next;                 ///<@private Next command in the queue.
    uint8_t*                   _response_buffer;      ///<@private Buffer for response data if response_descriptor != 0x00.
    union {
        timeout_type           _extra_timeout;        ///<@private If MIP_STATUS_PENDING:   Duration to wait for reply, excluding base timeout time from the queue object.
        timestamp_type         _timeout_time;         ///<@private If MIP_STATUS_WAITING:   timestamp_type after which the command will be timed out.
        timestamp_type         _reply_time;           ///<@private If MIP_STATUS_COMPLETED: timestamp_type from the packet containing the ack/nack.
    };
    uint8_t                    _descriptor_set;       ///<@private Command descriptor set.
    uint8_t                    _field_descriptor;     ///<@private Command field descriptor.
    uint8_t                    _response_descriptor;  ///<@private Response field descriptor, or 0x00 if no response field expected.
    union {
        uint8_t                _response_buffer_size; ///<@private If status < MIP_STATUS_COMPLETED, the size of the reply data buffer.
        uint8_t                _response_length;      ///<@private If status == MIP_STATUS_COMPLETED, the length of the reply data.
    };
    volatile mip_cmd_result    _status;               ///<@private The current status of the command. Writing this to any MipAck value may cause deallocation.
};

void mip_pending_cmd_init(struct mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor);
void mip_pending_cmd_init_with_timeout(struct mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, timeout_type additional_time);
void mip_pending_cmd_init_with_response(struct mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_buffer_size);
void mip_pending_cmd_init_full(struct mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_size, timeout_type additional_time);

mip_cmd_result mip_pending_cmd_status(const struct mip_pending_cmd* cmd);

const uint8_t* mip_pending_cmd_response(const struct mip_pending_cmd* cmd);
uint8_t mip_pending_cmd_response_length(const struct mip_pending_cmd* cmd);

bool mip_pending_cmd_check_timeout(const struct mip_pending_cmd* cmd, timestamp_type now);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup CommandQueue  mip_cmd_queue functions
///
///@{

struct mip_cmd_queue
{
    struct mip_pending_cmd* _first_pending_cmd;
    timeout_type            _base_timeout;
};

void mip_cmd_queue_init(struct mip_cmd_queue* queue, timeout_type base_reply_timeout);
void mip_cmd_queue_enqueue(struct mip_cmd_queue* queue, struct mip_pending_cmd* cmd);
void mip_cmd_queue_dequeue(struct mip_cmd_queue* queue, struct mip_pending_cmd* cmd);

void mip_cmd_queue_update(struct mip_cmd_queue* queue, timestamp_type timestamp);

void mip_cmd_queue_set_base_reply_timeout(struct mip_cmd_queue* queue, timeout_type timeout);
timeout_type mip_cmd_queue_base_reply_timeout(const struct mip_cmd_queue* queue);

struct mip_packet;

void mip_cmd_queue_process_packet(struct mip_cmd_queue* queue, const struct mip_packet* packet, timestamp_type timestamp);


///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif
