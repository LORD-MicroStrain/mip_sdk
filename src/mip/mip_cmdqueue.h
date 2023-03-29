
#pragma once

#include <stdint.h>

#include "mip_types.h"
#include "mip_result.h"
#include "mip_packet.h"

#include "utils/mip_mutex.h"

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{

////////////////////////////////////////////////////////////////////////////////
///@defgroup MipCommandQueue_c Mip Command Queue [C]
///
///@brief Functions for handling command responses.
///
///@{


////////////////////////////////////////////////////////////////////////////////
///@defgroup PendingCommand  mip_pending_cmd functions [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a command awaiting a reply from the device.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///

typedef struct mip_pending_cmd
{
    struct mip_pending_cmd*     _next;                 ///<@private Next command in the queue.
    uint8_t*                    _response_buffer;      ///<@private Buffer for response data if response_descriptor != 0x00.
    union {                                            ///<@private
        timeout_type            _extra_timeout;        ///<@private If MIP_STATUS_PENDING:   Duration to wait for reply, excluding base timeout time from the queue object.
        timestamp_type          _timeout_time;         ///<@private If MIP_STATUS_WAITING:   timestamp_type after which the command will be timed out.
        timestamp_type          _reply_time;           ///<@private If MIP_STATUS_COMPLETED: timestamp_type from the packet containing the ack/nack.
    };
    uint8_t                     _descriptor_set;       ///<@private Command descriptor set.
    uint8_t                     _field_descriptor;     ///<@private Command field descriptor.
    uint8_t                     _response_descriptor;  ///<@private Response field descriptor, or 0x00 if no response field expected.
    union {
        uint8_t                 _response_buffer_size; ///<@private If status < MIP_STATUS_COMPLETED, the size of the reply data buffer.
        uint8_t                 _response_length;      ///<@private If status == MIP_STATUS_COMPLETED, the length of the reply data.
    };                                                 ///<@private
    volatile enum mip_cmd_result _status;              ///<@private The current status of the command. Writing this to any MipAck value may cause deallocation.

//#ifdef MIP_ENABLE_THREADING
//    mip_thread_signal           _signal;               ///<@private Used to signal/wake up a waiting thread when the command has completed.
//#endif

} mip_pending_cmd;

void mip_pending_cmd_init(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor);
void mip_pending_cmd_init_with_timeout(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, timeout_type additional_time);
void mip_pending_cmd_init_with_response(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_buffer_size);
void mip_pending_cmd_init_full(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_size, timeout_type additional_time);

enum mip_cmd_result mip_pending_cmd_status(const mip_pending_cmd* cmd);
uint8_t mip_pending_cmd_response_descriptor(const mip_pending_cmd* cmd);
const uint8_t* mip_pending_cmd_response(const mip_pending_cmd* cmd);
uint8_t mip_pending_cmd_response_length(const mip_pending_cmd* cmd);

int mip_pending_cmd_remaining_time(const mip_pending_cmd* cmd, timestamp_type now);
bool mip_pending_cmd_check_timeout(const mip_pending_cmd* cmd, timestamp_type now);

void mip_pending_cmd_notify(mip_pending_cmd* cmd, mip_cmd_result status);

///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup CommandQueue  mip_cmd_queue functions [C]
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief Holds a list of pending commands.
///
/// Currently only one command may be pending at a time.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///

typedef struct mip_cmd_queue
{
    mip_pending_cmd* _first_pending_cmd;
    timeout_type     _base_timeout;
    uint8_t          _last_timeout_desc_set;
    uint8_t          _last_timeout_field_desc;

#ifdef MIP_ENABLE_THREADING
    mip_mutex_type   _mutex;
    mip_thread_signal _signal;
#endif // MIP_ENABLE_THREADING

#ifdef MIP_ENABLE_DIAGNOSTICS
    uint16_t         _diag_cmds_queued;    ///<@private Number of queued commands.
    uint16_t         _diag_cmds_acked;     ///<@private Number of successful commands.
    uint8_t          _diag_cmds_nacked;    ///<@private Number of commands failed by the device.
    uint8_t          _diag_cmds_timedout;  ///<@private Number of commands that have timed out.
    uint8_t          _diag_cmds_failed;    ///<@private Number of commands failed due to errors not from the device.
#endif // MIP_ENABLE_DIAGNOSTICS

} mip_cmd_queue;

void mip_cmd_queue_init(mip_cmd_queue* queue, timeout_type base_reply_timeout);
void mip_cmd_queue_deinit(mip_cmd_queue* queue);

void mip_cmd_queue_enqueue(mip_cmd_queue* queue, mip_pending_cmd* cmd);
bool mip_cmd_queue_dequeue(mip_cmd_queue* queue, mip_pending_cmd* cmd);
void mip_cmd_queue_cancel(mip_cmd_queue* queue, mip_pending_cmd* cmd);

void mip_cmd_queue_clear(mip_cmd_queue* queue);

void mip_cmd_queue_update(mip_cmd_queue* queue, timestamp_type timestamp);

void mip_cmd_queue_process_packet(mip_cmd_queue* queue, const mip_packet* packet, timestamp_type timestamp);

void mip_cmd_queue_set_base_reply_timeout(mip_cmd_queue* queue, timeout_type timeout);
timeout_type mip_cmd_queue_base_reply_timeout(const mip_cmd_queue* queue);


#ifdef MIP_ENABLE_DIAGNOSTICS
uint16_t mip_cmd_queue_diagnostic_cmds_queued(const mip_cmd_queue* queue);
uint16_t mip_cmd_queue_diagnostic_cmds_failed(const mip_cmd_queue* queue);
uint16_t mip_cmd_queue_diagnostic_cmds_successful(const mip_cmd_queue* queue);

uint16_t mip_cmd_queue_diagnostic_cmd_acks(const mip_cmd_queue* queue);
uint16_t mip_cmd_queue_diagnostic_cmd_nacks(const mip_cmd_queue* queue);
uint16_t mip_cmd_queue_diagnostic_cmd_timeouts(const mip_cmd_queue* queue);
uint16_t mip_cmd_queue_diagnostic_cmd_errors(const mip_cmd_queue* queue);
#endif // MIP_ENABLE_DIAGNOSTICS

///@}
///@}
///@}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif
