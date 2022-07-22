#pragma once

#include "mip.hpp"
#include "mip_interface.h"

#include "definitions/descriptors.h"


namespace mscl
{


class MipDeviceInterface : public C::mip_interface
{
public:
    ///@copydoc mip_interface_init
    MipDeviceInterface(uint8_t* parseBuffer, size_t parseBufferSize, Timeout parseTimeout, Timeout baseReplyTimeout) { C::mip_interface_init(this, parseBuffer, parseBufferSize, parseTimeout, baseReplyTimeout); }

    MipDeviceInterface(const MipDeviceInterface&) = delete;
    MipDeviceInterface& operator=(const MipDeviceInterface&) = delete;

    void setMaxPacketsPerPoll(unsigned int maxPackets) { C::mip_interface_set_max_packets_per_update(this, maxPackets); }
    unsigned int maxPacketsPerPoll() const { return C::mip_interface_max_packets_per_update(this); }

    RemainingCount receiveBytes(const uint8_t* data, size_t length, Timestamp timestamp) { return C::mip_interface_receive_bytes(this, data, length, timestamp); }
    void processUnparsedPackets() { C::mip_interface_process_unparsed_packets(this); }

    Timeout baseReplyTimeout() const { return C::mip_cmd_queue_base_reply_timeout(&cmdQueue()); }
    void setBaseReplyTimeout(Timeout timeout) { C::mip_cmd_queue_set_base_reply_timeout(&cmdQueue(), timeout); }

    // bool sendToDevice(const uint8_t* data, size_t length) { return C::mip_interface_sendToDevice(this, data, length); }

    void receivePacket(const C::mip_packet& packet, Timestamp timestamp) { C::mip_interface_receive_packet(this, &packet, timestamp); }

    MipCmdResult waitForReply(const C::mip_pending_cmd& cmd) { return C::mip_interface_wait_for_reply(this, &cmd); }

    MipParser& parser() { return *static_cast<MipParser*>(C::mip_interface_parser(this)); }
    C::mip_cmd_queue& cmdQueue() { return *C::mip_interface_cmd_queue(this); }

    const MipParser& parser() const { return const_cast<MipDeviceInterface*>(this)->parser(); }
    const C::mip_cmd_queue& cmdQueue() const { return const_cast<MipDeviceInterface*>(this)->cmdQueue(); }

    template<class Function>
    bool parseFromSource(Function function, unsigned int maxPackets=MIPPARSER_UNLIMITED_PACKETS) { return parseMipDataFromSource(parser(), function, maxPackets); }

    bool sendToDevice(const mscl::C::mip_packet& packet) { return sendToDevice(mscl::C::mip_packet_pointer(&packet), mscl::C::mip_packet_total_length(&packet)); }

public:
    virtual bool update() = 0;
    virtual bool sendToDevice(const uint8_t* data, size_t length) = 0;
};



template<class Cmd>
MipCmdResult runCommand(C::mip_interface& device, const Cmd& cmd)
{
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    MipPacket packet = MipPacket::createFromField(buffer, sizeof(buffer), cmd);

    C::mip_pending_cmd pending;
    C::mip_pending_cmd_init(&pending, MipFieldInfo<Cmd>::descriptorSet, MipFieldInfo<Cmd>::fieldDescriptor);

    return C::mip_interface_run_command_packet(&device, &packet, &pending);
}

template<class Cmd>
MipCmdResult runCommand(C::mip_interface& device, const Cmd& cmd, typename MipFieldInfo<Cmd>::Response& response)
{
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    MipPacket packet = MipPacket::createFromField(buffer, sizeof(buffer), cmd);

    C::mip_pending_cmd pending;
    C::mip_pending_cmd_init_with_response(&pending, MipFieldInfo<Cmd>::descriptorSet, MipFieldInfo<Cmd>::fieldDescriptor, MipFieldInfo<Cmd>::responseDescriptor, buffer, MIP_FIELD_PAYLOAD_LENGTH_MAX);

    MipCmdResult result = C::mip_interface_run_command_packet(&device, &packet, &pending);
    if( result != C::MIP_ACK_OK )
        return result;

    size_t responseLength = C::mip_pending_cmd_response_length(&pending);
    size_t offset = MipFieldInfo<Cmd>::extract_response(buffer, responseLength, 0, response);
    if( offset != responseLength )
        return C::MIP_STATUS_ERROR;

    return C::MIP_ACK_OK;
}


} // namespace mscl
