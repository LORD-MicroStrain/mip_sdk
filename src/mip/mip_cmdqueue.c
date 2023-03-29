
#include "mip_cmdqueue.h"

#include "mip_field.h"
#include "mip_packet.h"
#include "mip_logging.h"
#include "utils/mip_mutex.h"
#include "definitions/descriptors.h"

#include <string.h>
#include <assert.h>


#define MIP_REPLY_DESC_GLOBAL_ACK_NACK 0xF1

#define MIP_INDEX_REPLY_DESCRIPTOR 0
#define MIP_INDEX_REPLY_ACK_CODE   1


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending command with no reponse data or additional time.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///
void mip_pending_cmd_init(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor)
{
    mip_pending_cmd_init_full(cmd, descriptor_set, field_descriptor, 0x00, NULL, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with extra timeout time.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///@param additional_time
///       Additional time on top of the base reply timeout for this specific command.
///
void mip_pending_cmd_init_with_timeout(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, timeout_type additional_time)
{
    mip_pending_cmd_init_full(cmd, descriptor_set, field_descriptor, 0x00, NULL, 0, additional_time);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with expected response data.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///@param response_descriptor
///       Optional response data descriptor. Use 0x00 if no data is expected.
///@param response_buffer
///       Optional buffer to hold response data, if any. If NULL, response_buffer_size must be 0.
///@param response_buffer_size
///       Optional buffer to hold response data, if any. If NULL, response_buffer_size must be 0.
///@param response_buffer_size
///       Size of the response buffer. The response will be limited to this size.
///
void mip_pending_cmd_init_with_response(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_buffer_size)
{
    mip_pending_cmd_init_full(cmd, descriptor_set, field_descriptor, response_descriptor, response_buffer, response_buffer_size, 0);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize a pending mip commmand with all parameters.
///
///@param cmd
///@param descriptor_set
///       Command descriptor set.
///@param field_descriptor
///       Command field descriptor.
///@param response_descriptor
///       Optional response data descriptor. Use 0x00 if no data is expected.
///@param response_buffer
///       Optional buffer to hold response data, if any. If NULL, response_buffer_size must be 0.
///@param response_buffer_size
///       Size of the response buffer. The response will be limited to this size.
///@param additional_time
///       Additional time on top of the base reply timeout for this specific command.
///
void mip_pending_cmd_init_full(mip_pending_cmd* cmd, uint8_t descriptor_set, uint8_t field_descriptor, uint8_t response_descriptor, uint8_t* response_buffer, uint8_t response_buffer_size, timeout_type additional_time)
{
    cmd->_next                 = NULL;
    cmd->_response_buffer      = NULL;
    cmd->_extra_timeout        = additional_time;
    cmd->_descriptor_set       = descriptor_set;
    cmd->_field_descriptor     = field_descriptor;
    cmd->_response_descriptor  = response_descriptor;
    cmd->_response_buffer      = response_buffer;
    cmd->_response_buffer_size = response_buffer_size;
    // cmd->_ack_code            = 0xFF; // invalid
    cmd->_status               = MIP_STATUS_NONE;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Returns the status of the pending command.
///
///@see mip_cmd_status
///
enum mip_cmd_result mip_pending_cmd_status(const mip_pending_cmd* cmd)
{
    return cmd->_status;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the response descriptor.
///
uint8_t mip_pending_cmd_response_descriptor(const mip_pending_cmd* cmd)
{
    return cmd->_response_descriptor;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the response payload pointer.
///
/// This function may only be called after the command finishes with an ACK.
///
const uint8_t* mip_pending_cmd_response(const mip_pending_cmd* cmd)
{
    assert(mip_cmd_result_is_finished(cmd->_status));

    return cmd->_response_buffer;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Returns the length of the response data.
///
/// This function may only be called after the command finishes.
/// If the command completed with a NACK, or if it timed out, the response
/// length will be zero.
///
uint8_t mip_pending_cmd_response_length(const mip_pending_cmd* cmd)
{
    assert(mip_cmd_result_is_finished(cmd->_status));

    return cmd->_response_length;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines how much time is remaining before the command times out.
///
/// For most cases you should instead call mip_pending_cmd_check_timeout() to
/// know if the command has timed out or not.
///
///@param cmd The command to check. Must be in MIP_STATUS_WAITING state.
///@param now The current timestamp.
///
///@warning The command must be in the MIP_STATUS_WAITING state, otherwise the
///         result is unspecified.
///
///@returns The time remaining before the command times out. The value will be
///         negative if the timeout time has passed.
///
int mip_pending_cmd_remaining_time(const mip_pending_cmd* cmd, timestamp_type now)
{
    assert(cmd->_status == MIP_STATUS_WAITING);

    // result <= 0 if timed out.
    // Note: this still works with unsigned overflow.
    return (int)(now - cmd->_timeout_time);
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
bool mip_pending_cmd_check_timeout(const mip_pending_cmd* cmd, timestamp_type now)
{
    if( cmd->_status == MIP_STATUS_WAITING )
    {
        if( mip_pending_cmd_remaining_time(cmd, now) > 0 )
        {
            return true;
        }
    }

    return false;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Signal a pending command that it has finished.
///
/// This is used to signal the waiting function (mip_interface_wait_for_reply)
/// that the command has finished due to reply, timeout, or cancellation.
///
///@warning This function may cause deallocation of the pending command struct
///         under certain circumstatnces - in particular, when the command was
///         issued from another thread (regardless of MIP_ENABLE_THREADING), it
///         may return from the command function, causing the pending cmd to go
///         out of scope.
///
///@warning This function must not be called from user code while the command
///         is in a command queue. Doing so will result in undefined behavior.
///         Instead, use mip_cmd_queue_cancel() which will call this function
///         internally with MIP_STATUS_CANCELLED.
///
///@param cmd
///
///@param status
///       The final command status. This should be the finished result, i.e.
///       mip_cmd_result_is_finished(status) should return true.
///
void mip_pending_cmd_notify(mip_pending_cmd* cmd, mip_cmd_result status)
{
    assert(mip_cmd_result_is_finished(status));

    MIP_MUTEX_LOCK(&cmd->_signal.mutex);

    cmd->_status = status;

    MIP_THREAD_SIGNAL(&cmd->_signal);

    MIP_MUTEX_UNLOCK(&cmd->_signal.mutex);
}


////////////////////////////////////////////////////////////////////////////////
///@brief Initializes a command queue.
///
///@param queue
///@param base_reply_timeout
///       The minimum timeout given to all MIP commands. Additional time may be
///       given to specific commands which take longer. This is intended to be
///       used to accommodate long communication latencies, such as when using
///       a TCP connection.
///
void mip_cmd_queue_init(mip_cmd_queue* queue, timeout_type base_reply_timeout)
{
    queue->_first_pending_cmd = NULL;
    queue->_base_timeout = base_reply_timeout;
    queue->_last_timeout_desc_set   = MIP_INVALID_DESCRIPTOR_SET;
    queue->_last_timeout_field_desc = MIP_INVALID_FIELD_DESCRIPTOR;

    MIP_DIAG_ZERO(queue->_diag_cmds_queued);
    MIP_DIAG_ZERO(queue->_diag_cmds_acked);
    MIP_DIAG_ZERO(queue->_diag_cmds_nacked);
    MIP_DIAG_ZERO(queue->_diag_cmds_timedout);
    MIP_DIAG_ZERO(queue->_diag_cmds_failed);

    MIP_MUTEX_INIT(&queue->_mutex);
}

////////////////////////////////////////////////////////////////////////////////
///@brief De-initialize a command queue.
///
/// Call this to clean up the command queue. Queued commands will be terminated
/// with MIP_STATUS_CANCELLED. If multithreading is enabled, the mutex is
/// destroyed.
///
///@see mip_cmd_queue_clear.
///
void mip_cmd_queue_deinit(mip_cmd_queue* queue)
{
    mip_cmd_queue_clear(queue);

    MIP_MUTEX_DEINIT(&queue->_mutex);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Queue a command to wait for replies.
///
/// If multithreading support (MIP_ENABLE_THREADING) is disabled:
///@li The update function cannot interrupt mip_cmd_queue_enque(), or
///@li Only one command is queued at a time.
///
///@param queue
///@param cmd Listens for replies to this command.
///
///@warning The command must not be deallocated or go out of scope while the
///         mip_cmd_status_is_finished returns false.
///
void mip_cmd_queue_enqueue(mip_cmd_queue* queue, mip_pending_cmd* cmd)
{
    // Sanity check: cmd should not have a next queue element.
    assert(cmd->_next == NULL);

    cmd->_status = MIP_STATUS_PENDING;

    MIP_MUTEX_LOCK(&queue->_mutex);

    if( queue->_first_pending_cmd )
    {
#ifndef MIP_ENABLE_THREADING
        MIP_LOG_DEBUG("Attempting to queue more than one command without MIP_ENABLE_THREADING - this is unsafe if the update is run from another thread.");
//        cmd->status = MIP_STATUS_CANCELLED;
//        return;
#endif

        mip_pending_cmd* tail = queue->_first_pending_cmd;

        // Sanity check: cmd is not already in the queue.
        assert(tail != cmd);

        while(tail->_next)
        {
            tail = tail->_next;

             // Sanity check: cmd is not already in the queue.
            assert(tail != cmd);
        }

        tail->_next = cmd;
    }
    else
        queue->_first_pending_cmd = cmd;

    cmd->_next = NULL;

    MIP_DIAG_INC(queue->_diag_cmds_queued, 1);

    MIP_MUTEX_UNLOCK(&queue->_mutex);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Removes a pending command from the queue without cancelling it.
///
/// This is a no-op if the command is not in this queue.
///
/// Thread-safe if MIP_ENABLE_THREADING is enabled.
///
///@internal
///
///@param queue
///@param cmd
///
///@returns true if the command was dequeued, or false if it wasn't in the queue.
///
bool mip_cmd_queue_dequeue(mip_cmd_queue* queue, mip_pending_cmd* cmd)
{
    bool found = false;

    MIP_MUTEX_LOCK(&queue->_mutex);

    if(queue->_first_pending_cmd == cmd)
    {
        queue->_first_pending_cmd = cmd->_next;
        cmd->_next = NULL;
        found = true;
    }
    else
    {
        // Search the queue and unlink the command if it's found.
        for(mip_pending_cmd* item = queue->_first_pending_cmd; item; item=item->_next)
        {
            if(item->_next == cmd)
            {
                item->_next = cmd->_next;
                cmd->_next = NULL;
                found = true;
                break;
            }

            // Sanity check that the queue is not somehow circular.
            assert(item != queue->_first_pending_cmd);
        }
    }

    MIP_MUTEX_UNLOCK(&queue->_mutex);

    return found;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Removes a pending command from the queue (if present) and cancels it.
///
/// Thread-safe if MIP_ENABLE_THREADING is enabled (see mip_cmd_queue_dequeue).
///
///@internal
///
///@param queue
///@param cmd
///
void mip_cmd_queue_cancel(mip_cmd_queue* queue, mip_pending_cmd* cmd)
{
    mip_cmd_queue_dequeue(queue, cmd);

    mip_pending_cmd_notify(cmd, MIP_STATUS_CANCELLED);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Flushes the queue by terminating commands with MIP_STATUS_CANCELLED.
///
/// Thread-safe if MIP_ENABLE_THREADING is enabled.
///
///@param queue
///
void mip_cmd_queue_clear(mip_cmd_queue* queue)
{
    MIP_MUTEX_LOCK(&queue->_mutex);

    while( queue->_first_pending_cmd )
    {
        mip_pending_cmd* pending = queue->_first_pending_cmd;

        queue->_first_pending_cmd = pending->_next;

        // Ensure the removed cmds are fully unlinked to help avoid nasty surprises.
        // This is similar to setting a pointer to NULL after freeing it.
        pending->_next = NULL;

        // This may deallocate the pending command in another thread,
        // so make sure to dequeue the command first.
        mip_pending_cmd_notify(pending, MIP_STATUS_CANCELLED);
    }

    MIP_MUTEX_UNLOCK(&queue->_mutex);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Call periodically to make sure commands time out if no packets are
///       received.
///
/// Call this during the device update if no calls to mip_cmd_queue_process_packet
/// are made (e.g. because no packets were received). It is safe to call this
/// in either case.
///
///@param queue
///@param now
///
void mip_cmd_queue_update(mip_cmd_queue* queue, timestamp_type now)
{
    MIP_MUTEX_LOCK(&queue->_mutex);

    // +-----------+-----------+-----------+-----------+--
    // |  WAITING  |  WAITING  |  PENDING  |  PENDING  |
    // +-----------+-----------+-----------+-----------+--

    // Waiting commands get the timeout checked in this function because there might not be any packets arriving.
    // Only the first command in the queue can time out; out-of-order timeouts are not allowed.
    // All preceding commands must be processed by the device first, so a long-running command should not
    // cause a subsequent command to time out before the device has even had a chance to process it.
    while( (queue->_first_pending_cmd != NULL) && (queue->_first_pending_cmd->_status == MIP_STATUS_WAITING) )
    {
        mip_pending_cmd* pending = queue->_first_pending_cmd;

        if( mip_pending_cmd_check_timeout(pending, now) )
        {
            // Drop this cmd from the head of the queue.
            queue->_first_pending_cmd = pending->_next;

            // Clear response length and mark when it timed out.
            pending->_response_length = 0;
            pending->_reply_time = now;

            // Set the last timed-out command descriptor.
            queue->_last_timeout_desc_set   = pending->_descriptor_set;
            queue->_last_timeout_field_desc = pending->_field_descriptor;

            // This must be last! Pending may be deallocated.
            mip_pending_cmd_notify(pending, MIP_STATUS_TIMEDOUT);

            MIP_DIAG_INC(queue->_diag_cmds_timedout, 1);
        }
    }

    //for(mip_pending_cmd* pending = queue->_first_pending_cmd; pending; pending = pending->_next )
    //{
    //    // PENDING commands get their timeout_time set and move to the WAITING state.
    //    if( pending->_status == MIP_STATUS_PENDING )
    //    {
    //        // Update the timeout to the timestamp of the timeout time.
    //        pending->_timeout_time = now + queue->_base_timeout + pending->_extra_timeout;
    //        pending->_status = MIP_STATUS_WAITING;
    //    }
    //}

    MIP_MUTEX_UNLOCK(&queue->_mutex);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Completes the pending command given a reply field.
///
///@warning The pending command may be deallocated.
///
///@param pending
///       The pending command (may be deallocated as a result!)
///@param[in,out] field
///       The mip field in the packet. The field pointer may be advanced to the
///       next field if reponse data is consumed.
///@param timestamp
///       Timestamp of the received packet.
///
///@returns The status of the pending command. This will always be a
///         "reply code" status.
///
static mip_cmd_result mip_pending_cmd_process_reply(mip_pending_cmd* pending, /*not const*/ mip_field* field, timestamp_type timestamp)
{
    // Payload is already validated by mip_cmd_queue_process_reply.
    const uint8_t ack_code = mip_field_payload(field)[MIP_INDEX_REPLY_ACK_CODE];

    uint8_t response_length = 0;
    mip_field response_field;

    // ------+------+------+------+------+------+------+------+------+------------------------
    //  ...  | 0x02 | 0xF1 | cmd1 | nack | 0x02 | 0xF1 | cmd2 |  ack |  response field ...
    // ------+------+------+------+------+------+------+------+------+------------------------

    // If the command was ACK'd, check if response data is expected.
    if( pending->_response_descriptor != 0x00 && ack_code == MIP_ACK_OK )
    {
        // Look ahead one field for response data.
        response_field = mip_field_next_after(&field);
        if( mip_field_is_valid(&response_field) )
        {
            const uint8_t response_descriptor = mip_field_field_descriptor(&response_field);

            // This is a wildcard to accept any response data descriptor.
            // Needed when the response descriptor is not known or is wrong.
            if( pending->_response_descriptor == MIP_REPLY_DESC_GLOBAL_ACK_NACK )
                pending->_response_descriptor = response_descriptor;

            // Make sure the response descriptor matches what is expected.
            if( response_descriptor == pending->_response_descriptor )
            {
                // Update the response_size field to reflect the actual size.
                response_length = mip_field_payload_length(&response_field);

                // Skip this field when iterating for next ack/nack reply.
                // This assigns the struct metadata which points at the field, and not the actual field data in the packet.
                *field = response_field;
            }
        }
    }

    // Limit response data size to lesser of buffer size or actual response length (zero if nack or no data).
    pending->_response_length = (response_length < pending->_response_buffer_size) ? response_length : pending->_response_buffer_size;

    // Copy response data to the pending buffer (skip if response_field is invalid).
    if( pending->_response_length > 0 )
        memcpy(pending->_response_buffer, mip_field_payload(&response_field), pending->_response_length);

    pending->_reply_time = timestamp;  // Completion time

    // This may deallocate the pending command - do it last!
    mip_pending_cmd_notify(pending, ack_code);

    return (mip_cmd_result)ack_code;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Processes a MIP command reply field by attempting to match it with a
///       queued pending command.
///
///@param queue
///
///@param[in,out] field
///       The mip field in the packet. The field pointer may be advanced to the
///       next field if reponse data is consumed.
///@param timestamp
///       Timestamp of the received packet.
///
///@returns MIP_STATUS_NONE if no matching command was found.
///@returns A mip reply code if a matching command was found.
///
static mip_cmd_result mip_cmd_queue_process_reply(mip_cmd_queue* queue, /*not const*/ mip_field* field, timestamp_type timestamp)
{
    const uint8_t descriptor_set = mip_field_descriptor_set(field);
    const uint8_t* const reply_payload = mip_field_payload(field);

    const uint8_t cmd_descriptor = reply_payload[MIP_INDEX_REPLY_DESCRIPTOR];

    const bool might_be_stale_reply = (
        (descriptor_set == queue->_last_timeout_desc_set) &&
        (cmd_descriptor == queue->_last_timeout_field_desc)
    );

    // Iterate the pending command queue looking for the request to this reply.
    //
    // If the reply is not for the first thing in the queue, either:
    // A) it's a stale reply for a previously command that timed out,
    // B) the command (or reply) got lost and this reply goes with one of the following commands, or
    // C) the reply goes to something else and the command queue isn't aware of it.
    //
    // The control logic works as follows, assuming the queue contains commands A,B,C,...:
    // (Down is YES, right is NO)
    //
    // reply --> A? -----> B? -----------------> C? --------------> ... ---> Stop
    //           |         |                     |
    //           V         V                     V
    //         A done,   stale? --> B done,    stale? --> C done,
    //          Stop       |        A T.O.,      |        A T.O.,
    //                     V        Stop         V        B T.O.,
    //                    Stop                  Stop      Stop
    //
    mip_pending_cmd* prev_pending = NULL;
    for(mip_pending_cmd* pending = queue->_first_pending_cmd; pending != NULL; pending = pending->_next )
    {
        // If the command in the queue matches the reply descriptor, process the reply.
        // In the above diagram, this implements the line at the top going from left to right.
        if( pending->_descriptor_set == descriptor_set && pending->_field_descriptor == cmd_descriptor )
        {
            if( might_be_stale_reply )
            {
                // Assume that a match to the first thing in the queue is not a stale reply.
                //
                // If there are two identical commands in a row, say "AA", and one times out, it's
                // impossible to know which was which without contextual information.
                // Therefore, assume the user has set appropriate timeouts and that the previous
                // command really did time out; that this reply goes with the newer one.
                //
                if(pending != queue->_first_pending_cmd)
                    return MIP_STATUS_NONE;

                // fall through if first queued command.
            }

            // Pop all preceding elements off the queue.
            for(mip_pending_cmd* skipped = queue->_first_pending_cmd; skipped != pending; )
            {
                assert(skipped->_status == MIP_STATUS_WAITING); // Sanity check: queue has only waiting commands up to this point.

                mip_pending_cmd* tmp = skipped;
                skipped = skipped->_next;

                // This may deallocate tmp, so make sure "skipped" has been moved to the next element.
                mip_pending_cmd_notify(tmp, MIP_STATUS_TIMEDOUT);

                MIP_DIAG_INC(queue->_diag_cmds_timedout, 1);
            }

            // Advance the queue to the next pending command.
            queue->_first_pending_cmd = pending->_next;

            // Process the ack/nack and any reply data.
            // This may cause "pending" to be deallocated.
            // This may advance the mip field.
            mip_cmd_result result = mip_pending_cmd_process_reply(pending, field, timestamp);

            if( mip_cmd_result_is_ack(result) )
                MIP_DIAG_INC(queue->_diag_cmds_acked, 1);
            else
                MIP_DIAG_INC(queue->_diag_cmds_nacked, 1);

            return result;
        }
    }

    return MIP_STATUS_NONE;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Process an incoming packet and check for replies to pending commands.
///
/// Call this from the Mip_parser callback, passing the arguments directly.
///
///@param queue
///@param packet The received MIP packet. Assumed to be valid.
///@param timestamp The time the packet was received
///
void mip_cmd_queue_process_packet(mip_cmd_queue* queue, const mip_packet* packet, timestamp_type timestamp)
{
    // Check if the packet is a command descriptor set.
    const uint8_t descriptor_set = mip_packet_descriptor_set(packet);
    if( !mip_is_cmd_descriptor_set(descriptor_set) )
        return;

    MIP_MUTEX_LOCK(&queue->_mutex);

    mip_field field = {0};
    while( mip_field_next_in_packet(&field, packet) )
    {
        if( mip_field_field_descriptor(&field) != MIP_REPLY_DESC_GLOBAL_ACK_NACK )
            continue;

        if( mip_field_payload_length(&field) != 2 )
            continue;

        mip_cmd_queue_process_reply(queue, &field, timestamp);
    }

    MIP_MUTEX_UNLOCK(&queue->_mutex);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Sets the base reply timeout for all commands.
///
/// The base reply timeout is the minimum time to wait for a reply.
/// Takes effect for any commands queued after this function call.
///
///@param queue
///@param timeout
///
void mip_cmd_queue_set_base_reply_timeout(mip_cmd_queue* queue, timeout_type timeout)
{
    queue->_base_timeout = timeout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the base reply timeout for all commands.
///
///@returns The minimum time to wait for a reply to any command.
///
timeout_type mip_cmd_queue_base_reply_timeout(const mip_cmd_queue* queue)
{
    return queue->_base_timeout;
}


#ifdef MIP_ENABLE_DIAGNOSTICS

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands ever put into the queue.
///
/// In most cases this is the number of commands sent to the device.
///
uint16_t mip_cmd_queue_diagnostic_cmds_queued(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_queued;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands that have failed for any reason.
///
uint16_t mip_cmd_queue_diagnostic_cmds_failed(const mip_cmd_queue* queue)
{
    return (uint16_t)queue->_diag_cmds_nacked + queue->_diag_cmds_failed + queue->_diag_cmds_timedout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of successful commands.
///
/// Same as mip_cmd_queue_diagnostic_cmd_acks().
///
uint16_t mip_cmd_queue_diagnostic_cmds_successful(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_acked;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of successful commands.
///
/// Same as mip_cmd_queue_diagnostic_cmds_successful().
///
uint16_t mip_cmd_queue_diagnostic_cmd_acks(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_acked;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands nack'd by the device.
///
uint16_t mip_cmd_queue_diagnostic_cmd_nacks(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_nacked;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of commands that did not receive a reply within the
///       time limit.
///
uint16_t mip_cmd_queue_diagnostic_cmd_timeouts(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_timedout;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Gets the number of command errors not caused by the device.
///
uint16_t mip_cmd_queue_diagnostic_cmd_errors(const mip_cmd_queue* queue)
{
    return queue->_diag_cmds_failed;
}

#endif // MIP_ENABLE_DIAGNOSTICS
