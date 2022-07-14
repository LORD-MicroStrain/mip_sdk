#pragma once

#include "mip.hpp"
#include "mip_interface.h"

#include "definitions/descriptors.h"


namespace mscl
{


class MipDeviceInterface : public C::MipInterfaceState
{
public:
    ///@copydoc MipInterface_init
    MipDeviceInterface(uint8_t* parseBuffer, size_t parseBufferSize, Timeout parseTimeout, Timeout baseReplyTimeout) { C::MipInterface_init(this, parseBuffer, parseBufferSize, parseTimeout, baseReplyTimeout); }

    MipDeviceInterface(const MipDeviceInterface&) = delete;
    MipDeviceInterface& operator=(const MipDeviceInterface&) = delete;

    void setMaxPacketsPerPoll(unsigned int maxPackets) { C::MipInterface_setMaxPacketsPerPoll(this, maxPackets); }
    unsigned int maxPacketsPerPoll() const { return C::MipInterface_maxPacketsPerPoll(this); }

    RemainingCount receiveBytes(const uint8_t* data, size_t length, Timestamp timestamp) { return C::MipInterface_receiveBytes(this, data, length, timestamp); }
    void processUnparsedPackets() { C::MipInterface_processUnparsedPackets(this); }

    // bool sendToDevice(const uint8_t* data, size_t length) { return C::MipInterface_sendToDevice(this, data, length); }

    void receivePacket(const C::MipPacket& packet, Timestamp timestamp) { C::MipInterface_receivePacket(this, &packet, timestamp); }

    MipCmdResult waitForReply(const C::MipPendingCmd& cmd) { return C::MipInterface_waitForReply(this, &cmd); }

    C::MipParsingState* parser() { return C::MipInterface_parser(this); }
    C::MipCmdQueue* cmdQueue() { return C::MipInterface_cmdQueue(this); }

    template<class Function>
    bool parseFromSource(Function function, unsigned int maxPackets=MIPPARSER_UNLIMITED_PACKETS) { return parseMipDataFromSource(*parser(), function, maxPackets); }

    bool sendToDevice(const mscl::C::MipPacket& packet) { return sendToDevice(mscl::C::MipPacket_pointer(&packet), mscl::C::MipPacket_totalLength(&packet)); }

public:
    virtual bool poll() = 0;
    virtual bool sendToDevice(const uint8_t* data, size_t length) = 0;
};



template<class Cmd>
MipCmdResult runCommand(C::MipInterfaceState* device, const Cmd& cmd)
{
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    MipPacket packet = MipPacket::createFromField(buffer, sizeof(buffer), cmd);

    C::MipPendingCmd pending;
    C::MipPendingCmd_init(&pending, MipFieldInfo<Cmd>::descriptorSet, MipFieldInfo<Cmd>::fieldDescriptor);
    C::MipCmdQueue_enqueue(C::MipInterface_cmdQueue(device), &pending);

    return C::MipInterface_runCommandPacket(device, &packet, &pending);
}

template<class Cmd>
MipCmdResult runCommand(C::MipInterfaceState* device, const Cmd& cmd, typename MipFieldInfo<Cmd>::Response& response)
{
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    MipPacket packet = MipPacket::createFromField(buffer, sizeof(buffer), cmd);

    C::MipPendingCmd pending;
    C::MipPendingCmd_initWithResponse(&pending, MipFieldInfo<Cmd>::descriptorSet, MipFieldInfo<Cmd>::fieldDescriptor, MipFieldInfo<Cmd>::responseDescriptor, buffer, MIP_FIELD_PAYLOAD_LENGTH_MAX);

    MipCmdResult result = C::MipInterface_runCommandPacket(device, &packet, &pending);
    if( result != MIP_ACK_OK )
        return result;

    size_t responseLength = C::MipPendingCmd_responseLength(&pending);
    size_t offset = MipFieldInfo<Cmd>::extract_response(buffer, responseLength, 0, response);
    if( offset != responseLength )
        return MIP_STATUS_ERROR;

    return MIP_ACK_OK;
}


} // namespace mscl
