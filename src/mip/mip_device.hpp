#pragma once

#include "mip.hpp"
#include "mip_interface.h"


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

    C::MipCmdResult waitForReply(const C::MipPendingCmd& cmd) { return C::MipInterface_waitForReply(this, &cmd); }

    C::MipParsingState* parser() { return C::MipInterface_parser(this); }
    C::MipCmdQueue* cmdQueue() { return C::MipInterface_cmdQueue(this); }

    template<class Function>
    bool parseFromSource(Function function, unsigned int maxPackets=MIPPARSER_UNLIMITED_PACKETS) { return parseMipDataFromSource(*parser(), function, maxPackets); }

public:
    virtual bool poll() = 0;
    virtual bool sendToDevice(const uint8_t* data, size_t length) = 0;
};


} // namespace mscl
