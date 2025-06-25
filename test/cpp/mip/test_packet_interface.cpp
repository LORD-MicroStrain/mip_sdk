
#include "test.h"

#include <microstrain/span.hpp>
#include <mip/mip_packet.hpp>

#include <mip/definitions/data_sensor.hpp>
#include <mip/definitions/data_shared.hpp>
#include <mip/definitions/commands_base.hpp>

// A ping command
const uint8_t PING[] = { 0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6 };
// Sample data field
const mip::data_sensor::ScaledAccel ACCEL_FIELD = { {1.f, 2.f, -3.f} };
const uint8_t ACCEL_PKT[] = { 0x75, 0x65, 0x80, 0x0e, 0x0e, 0x04, 0x3f, 0x80, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x40, 0x00, 0x00, 0x79, 0xed };
//const mip::data_shared::ReferenceTimestamp REFTIME = { 1'234'567'890 };

void print_packet(const mip::PacketView& packet)
{
    print_buffer(stderr, packet.pointer(), packet.totalLength());
}

bool checkPacketView(const mip::PacketView& packet, microstrain::Span<const uint8_t> compare, const char* method)
{
    bool ok = true;

    ok &= check(packet.isSane(), "Insane packet from %s", method);
    ok &= check(packet.descriptorSet() == compare[mip::C::MIP_INDEX_DESCSET], "Wrong descriptor set from %s", method);
    ok &= check(packet.payloadLength() == compare[mip::C::MIP_INDEX_LENGTH], "Wrong payload length from %s", method);
    ok &= check(packet.totalLength() == packet.payloadLength()+mip::C::MIP_PACKET_LENGTH_MIN, "Wrong total length from %s", method);
    ok &= check_equal(packet.payload(), packet.pointer() + mip::C::MIP_INDEX_PAYLOAD, "Wrong payload pointer from %s", method);
    ok &= check(packet.pointer() != nullptr, "PacketRef shouldn't have NULL pointer from %s", method);
    ok &= check_equal(packet.totalSpan().data(), packet.pointer(), "totalSpan().data() should match pointer()");
    ok &= check_equal(packet.totalSpan().size(), packet.totalLength(), "totalSpan().size() should match totalLength()");
    ok &= check_equal(packet.payloadSpan().data(), packet.payload(), "payloadSpan().data() should match payload()");
    ok &= check_equal(packet.payloadSpan().size(), packet.payloadLength(), "payloadSpan().size() should match payloadLength()");
    ok &= check_equal(packet.remainingSpace(), packet.bufferSize()-packet.totalLength(), "remainingSpace() is wrong");

    if(std::memcmp(packet.pointer(), compare.data(), compare.size()) != 0)
    {
        fprintf(stderr, "Data mismatch from %s:\n", method);
        print_packet(packet);
        print_buffer(stderr, compare.data(), compare.size());
        ok = false;
    }

    return ok;
}

template<size_t Size>
bool checkPacketBuf(mip::SizedPacketBuf<Size>& packet, microstrain::Span<const uint8_t> compare, const char* method)
{
    bool ok = checkPacketView(packet, compare, method);

    ok &= check(packet.buffer() != nullptr, "NULL buffer from %s", method);
    ok &= check_equal(packet.bufferSize(), Size, "bufferSize() should match templated size from %s", method);
    ok &= check_equal(packet.pointer(), packet.buffer(), "Packet buffer/pointer mismatch from %s", method);
    ok &= check(packet.pointer() != compare.data(), "PacketBuf shouldn't point to original data buffer from %s", method);
    ok &= check_equal(packet.bufferSpan().data(), packet.buffer(), "BufferSpan().data() doesn't match .buffer()");
    ok &= check(packet.bufferSpan().size() == Size, "BufferSpan().data() doesn't match .buffer()");

    return ok;
}

bool checkPingPacketView(const mip::PacketView& packet, const char* method)
{
    return checkPacketView(packet, microstrain::Span<const uint8_t>(PING), method);
}

template<size_t Size>
bool checkPingPacketBuf(mip::SizedPacketBuf<Size>& packet, const char* method)
{
    return checkPacketBuf(packet, microstrain::Span<const uint8_t>(PING), method);
}


void testPacketView()
{
    //
    // Construction
    //

    uint8_t buffer[mip::PACKET_LENGTH_MAX];

    mip::PacketView packet1(buffer, sizeof(buffer), mip::commands_base::DESCRIPTOR_SET);
    checkPacketView(packet1, buffer, "PacketView initializing constructor");
    check(packet1.isEmpty(), "Packet should be empty after packetView initializing constructor");
    check_equal(packet1.bufferSize(), sizeof(buffer), "Wrong buffer size from PacketView initializing constructor");
    check_equal(packet1.remainingSpace(), mip::C::MIP_PACKET_PAYLOAD_LENGTH_MAX, "remainingSpace wrong after PacketView initializing constructor");
    //bool ok = packet1.addField(mip::commands_base::Ping::FIELD_DESCRIPTOR, nullptr, 0);
    //check(ok, "Failed to add field to packet1");
    //check_equal(packet1.remainingSpace(), mip::C::MIP_PACKET_PAYLOAD_LENGTH_MAX-2, "remainingSpace wrong after adding Ping command to packet1");
    //check_equal(packet1.bufferSize(), sizeof(buffer), "Wrong buffer size after adding Ping command to packet1");
    packet1.finalize();
    if(!check(packet1.isValid(), "Packet1 not valid after finalization"))
        print_packet(packet1);

    mip::PacketView packet2(PING, sizeof(PING));
    checkPingPacketView(packet2, "PacketView existing constructor");

    mip::C::mip_packet_view packet3c;
    mip::C::mip_packet_from_buffer(&packet3c, PING, sizeof(PING));
    mip::PacketView packet3(packet3c);
    checkPingPacketView(packet3, "PacketView C constructor");

    mip::PacketView packet4(microstrain::Span<uint8_t>(buffer), mip::commands_base::DESCRIPTOR_SET);
    checkPacketView(packet4, buffer, "PacketView initializing constructor (span version)");

    mip::PacketView packet5(microstrain::Span<const uint8_t>(PING, sizeof(PING)));
    checkPingPacketView(packet5, "PacketView existing constructor (span version)");


}

void testPacketBuf()
{
    //
    // Construction
    //

    // Default constructor
    mip::PacketBuf packet1;
    check( !mip::isValidDescriptorSet(packet1.descriptorSet()), "PacketBuf should default-construct to invalid descriptor set");

    // Specify descriptor set
    mip::PacketBuf packet2(mip::data_sensor::DESCRIPTOR_SET);
    check( packet2.descriptorSet() == mip::data_sensor::DESCRIPTOR_SET, "PacketBuf constructor with descriptor set doesn't work");

    // Construct from raw buffer, span, or existing view.
    mip::PacketBuf packet3(PING, sizeof(PING));
    mip::PacketBuf packet4(microstrain::Span<const uint8_t>(PING, sizeof(PING)));
    mip::PacketBuf packet5(mip::PacketView(const_cast<uint8_t*>(PING), sizeof(PING)));

    checkPingPacketBuf(packet3, "PacketBuf raw buffer constructor");
    checkPingPacketBuf(packet4, "PacketBuf span constructor");
    checkPingPacketBuf(packet5, "PacketBuf PacketView constructor");

    // Regular copy constructor
    mip::PacketBuf packet6(packet5);
    checkPingPacketBuf(packet6, "PacketBuf copy constructor");
    check(packet6.buffer() != packet5.buffer(), "Packet6 shouldn't point to packet5's data buffer from copy constructor");

    // Construct from SizedPacketBuf of differing size
    mip::SizedPacketBuf<8> packet7(packet5);
    checkPingPacketView(packet7, "SizedPacketBuf<8> copy constructor");

    // Construction from PacketView
    mip::PacketBuf packet8(packet5.ref());
    checkPingPacketBuf(packet8, "PacketBuf copy constructor (PacketView)");
    check(packet8.buffer() != packet5.buffer(), "Packet6 shouldn't point to packet5's data buffer from copy constructor");

    // Assignment
    mip::PacketBuf packet9;
    packet9 = packet5;
    checkPingPacketBuf(packet9, "PacketBuf operator=");
    check(packet9.buffer() != packet5.buffer(), "Packet9 shouldn't point to packet5's data buffer from operator=");

    // Create from field
    mip::PacketBuf packet10(ACCEL_FIELD);
    checkPacketBuf(packet10, ACCEL_PKT, "PacketBuf field constructor");

    // .ref() already tested
    // .buffer() already tested
    // .bufferSpan() already tested
    // .copyFrom tested by constructors

    // Test copying to destination
    uint8_t tmp[sizeof(PING)];
    packet5.copyPacketTo(tmp);
    check(std::memcmp(PING, tmp, sizeof(PING))==0, "Temporary buffer doesn't match after calling packet.copyPacketTo");

    std::memset(tmp, 0x00, sizeof(tmp));
    packet5.copyPacketTo(microstrain::Span<uint8_t>(tmp));
    check(std::memcmp(PING, tmp, sizeof(PING))==0, "Temporary buffer doesn't match after calling packet.copyPacketTo (span version)");

}


int main()
{
    testPacketView();
    testPacketBuf();

    return num_errors;
}
