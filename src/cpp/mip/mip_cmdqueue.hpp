#pragma once

#include <mip/mip_cmdqueue.h>

#include "mip_result.hpp"

#include <assert.h>
#include <cstring>


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{

////////////////////////////////////////////////////////////////////////////////
///@brief C++ wrapper around a command queue.
///
struct CmdQueue : public C::mip_cmd_queue
{
    CmdQueue(Timeout baseReplyTimeout=1000) { C::mip_cmd_queue_init(this, baseReplyTimeout); }
    ~CmdQueue() { assert(_first_pending_cmd==nullptr); }

    CmdQueue(const CmdQueue&) = delete;
    CmdQueue& operator=(const CmdQueue&) = delete;

    void enqueue(C::mip_pending_cmd& cmd) { C::mip_cmd_queue_enqueue(this, &cmd); }
    void dequeue(C::mip_pending_cmd& cmd) { C::mip_cmd_queue_dequeue(this, &cmd); }

    void clear() { C::mip_cmd_queue_clear(this); }

    void update(Timestamp now) { C::mip_cmd_queue_update(this, now); }

    void setBaseReplyTimeout(Timeout timeout) { C::mip_cmd_queue_set_base_reply_timeout(this, timeout); }
    Timeout baseReplyTimeout() const { return C::mip_cmd_queue_base_reply_timeout(this); }

    void processPacket(const C::mip_packet_view& packet, Timestamp timestamp) { C::mip_cmd_queue_process_packet(this, &packet, timestamp); }
};
static_assert(sizeof(CmdQueue) == sizeof(C::mip_cmd_queue), "CmdQueue must not have additional data members.");


////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing the state of a MIP command.
///
struct PendingCmd : public C::mip_pending_cmd
{
    ///@brief Create a null pending command in the CmdResult::NONE state.
    ///
    PendingCmd() { std::memset(static_cast<C::mip_pending_cmd*>(this), 0, sizeof(C::mip_pending_cmd)); }

    ///@brief Create a pending command for the given descriptor pair.
    ///
    ///@param descriptorSet   MIP descriptor set for the command.
    ///@param fieldDescriptor MIP field descriptor for the command.
    ///@param additionalTime  Optional additional time to allow for the device to process the command. Default 0.
    ///
    PendingCmd(uint8_t descriptorSet, uint8_t fieldDescriptor, Timeout additionalTime=0) { C::mip_pending_cmd_init_with_timeout(this, descriptorSet, fieldDescriptor, additionalTime); }

    ///@brief Create a pending command with expected response.
    ///
    ///@param descriptorSet      MIP descriptor set for the command.
    ///@param fieldDescriptor    MIP field descriptor for the command.
    ///@param responseDescriptor MIP field descriptor for the response.
    ///@param responseBuffer     A buffer used for the command response data. Must be big enough for the expected response. You can reuse the buffer used to send the command.
    ///@param responseBufferSize Length of responseBuffer in bytes.
    ///@param additionalTime     Optional additional time to allow for the device to process the command. Default 0.
    ///
    PendingCmd(uint8_t descriptorSet, uint8_t fieldDescriptor, uint8_t responseDescriptor, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime) { C::mip_pending_cmd_init_full(this, descriptorSet, fieldDescriptor, responseDescriptor, responseBuffer, responseBufferSize, additionalTime); }

    ///@brief Create a pending command given the actual command struct.
    ///
    ///@param cmd The C++ command struct (this must be the C++ version of the struct, the C struct will not work). It need not be fully populated; this parameter is unused except for its type information.
    ///@param additionalTime     Optional additional time to allow for the device to process the command. Default 0.
    ///
    template<class Cmd>
    PendingCmd(const Cmd& cmd, Timeout additionalTime=0) : PendingCmd(cmd.descriptorSet, cmd.fieldDescriptor, additionalTime) {}

    ///@brief Create a pending command given the actual command struct and a response buffer.
    ///
    ///@param cmd                The C++ command struct (this must be the C++ version of the struct, the C struct will not work). It need not be fully populated; this parameter is unused except for its type information.
    ///@param responseBuffer     A buffer used for the command response data. Must be big enough for the expected response. You can reuse the buffer used to send the command.
    ///@param responseBufferSize Length of responseBuffer in bytes.
    ///@param additionalTime     Optional additional time to allow for the device to process the command. Default 0.
    ///
    template<class Cmd>
    PendingCmd(const Cmd&, uint8_t* responseBuffer, uint8_t responseBufferSize, Timeout additionalTime=0) : PendingCmd(Cmd::DESCRIPTOR_SET, Cmd::FIELD_DESCRIPTOR, Cmd::Response::FIELD_DESCRIPTOR, responseBuffer, responseBufferSize, additionalTime) {}

    ///@brief Disable copying and moving. Once queued, a pending command must remain in the same memory location.
    ///
    PendingCmd(const PendingCmd&) = delete;
    PendingCmd& operator=(const PendingCmd&) = delete;

    ///@brief Sanity check that the PendingCmd is not deallocated while still in the queue.
    ///
    ~PendingCmd() { CmdResult tmp = status(); assert(tmp.isFinished() || tmp==CmdResult::STATUS_NONE); (void)tmp; }

    ///@brief Gets the status of the pending command.
    ///
    CmdResult status() const { return C::mip_pending_cmd_status(this); }

    ///@copydoc mip::C::mip_pending_cmd_response_descriptor
    uint8_t responseDescriptor() const { return C::mip_pending_cmd_response_descriptor(this); }

    ///@copydoc mip::C::mip_pending_cmd_response
    const uint8_t* response() const { return C::mip_pending_cmd_response(this); }

    ///@copydoc mip::C::mip_pending_cmd_response_length
    uint8_t responseLength() const { return C::mip_pending_cmd_response_length(this); }
};

///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
