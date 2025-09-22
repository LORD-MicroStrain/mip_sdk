#include <iomanip>
#include <ios>
#include <sstream>

#include <microstrain_test.hpp>
#include <microstrain/array_view.hpp>
#include <mip/mip_packet.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/data_sensor.hpp>
#include <mip/definitions/data_shared.hpp>

// A ping command
const uint8_t PING[] = { 0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6 };
// Sample data field
const mip::data_sensor::ScaledAccel ACCEL_FIELD = { {1.f, 2.f, -3.f} };
const uint8_t ACCEL_PKT[] = { 0x75, 0x65, 0x80, 0x0e, 0x0e, 0x04, 0x3f, 0x80, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xC0, 0x40, 0x00, 0x00, 0x79, 0xed };
//const mip::data_shared::ReferenceTimestamp REFTIME = { 1'234'567'890 };

std::string print_packet(const uint8_t *packet_data, const size_t size)
{
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0') << std::setw(2);

    for (size_t i = 0; i < size; ++i)
    {
        output << packet_data[i];
    }

    return output.str();
}

void checkPacketView(const mip::PacketView& packet, microstrain::ConstU8ArrayView compare, const char* method)
{
    FAIL_AND_LOG_IF_NOT_TRUE(packet.isSane(), "Insane packet from " << method);
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.descriptorSet(), compare[mip::C::MIP_INDEX_DESCSET], "Wrong descriptor set from " << method);
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.payloadLength(), compare[mip::C::MIP_INDEX_LENGTH], "Wrong payload length from " << method);
    // TODO: These contain domain leakage. Any calculations for the expected value should be removed and replaced
    //       with a hard-coded one. This will likely require restructuring how these tests work.
    //FAIL_AND_LOG_IF_NOT_EQUAL(packet.totalLength(), packet.payloadLength() + mip::C::MIP_PACKET_LENGTH_MIN,
    //    "Wrong total length from " << method);
    //FAIL_AND_LOG_IF_NOT_EQUAL(packet.payloadPointer(), packet.pointer() + mip::C::MIP_INDEX_PAYLOAD,
    //    "Wrong payload pointer from " << method);
    FAIL_AND_LOG_IF_EQUAL(packet.pointer(), nullptr, "Packet pointer was NULL from " << method);
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.data().data(), packet.pointer(), "Packet data and pointer were not the same");
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.data().size(), packet.totalLength(), "Packet size doesn't match total length");
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.payload().data(), packet.payloadPointer(), "Payload data doesn't match pointer");
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.payload().size(), packet.payloadLength(), "Payload size doesn't match the length");
    // TODO: This is domain leakage. Any calculations for the expected value should be removed and replaced
    //       with a hard-coded one. This will likely require restructuring how these tests work.
    //FAIL_AND_LOG_IF_NOT_EQUAL(packet.remainingSpace(), packet.bufferSize()-packet.packetLength(),
    //    "Remaining space calculation is wrong");

    int result = std::memcmp(packet.pointer(), compare.data(), compare.size());

    std::string actual_output = print_packet(packet.pointer(), packet.payloadLength());
    std::string expected_output = print_packet(compare.data(), compare.size());
    LOG_ON_FAIL(actual_output);
    LOG_ON_FAIL(expected_output);

    FAIL_AND_LOG_IF_NOT_EQUAL(result, 0, "Data mismatch from " << method);
}

template<size_t Size>
void checkPacketBuf(mip::SizedPacketBuf<Size>& packet, microstrain::ConstU8ArrayView compare, const char* method)
{
    checkPacketView(packet, compare, method);

    FAIL_AND_LOG_IF_EQUAL(packet.bufferPointer(), nullptr, "NULL buffer from " << method);
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.bufferSize(), Size, "Buffer size should match templated size from " << method);
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.pointer(), packet.bufferPointer(), "Packet buffer/pointer mismatch from " << method);
    FAIL_AND_LOG_IF_EQUAL(packet.pointer(), compare.data(), "PacketBuf shouldn't point to original data buffer from " << method);
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.buffer().data(), packet.bufferPointer(), "Buffer data doesn't match the buffer pointer");
    FAIL_AND_LOG_IF_NOT_EQUAL(packet.buffer().size(), Size, "Buffer size doesn't match Size");
}

/*
void checkPingPacketView(const mip::PacketView& packet, const char* method)
{
    checkPacketView(packet, PING, method);
}
*/

/*
template<size_t Size>
bool checkPingPacketBuf(mip::SizedPacketBuf<Size>& packet, const char* method)
{
    return checkPacketBuf(packet, PING, method);
}
*/


TEST("Packet view", "Packet view works as expected")
{
    uint8_t buffer[mip::PACKET_LENGTH_MAX];

    mip::PacketView packet1(buffer, sizeof(buffer), mip::commands_base::DESCRIPTOR_SET);
    checkPacketView(packet1, buffer, "PacketView initializing constructor");
    FAIL_AND_LOG_IF_NOT_TRUE(packet1.isEmpty(), "Packet should be empty after packetView initializing constructor");
    FAIL_AND_LOG_IF_NOT_EQUAL(packet1.buffer().size(), sizeof(buffer),
        "Wrong buffer size from PacketView initializing constructor");
    FAIL_AND_LOG_IF_NOT_EQUAL(packet1.remainingSpace(), mip::C::MIP_PACKET_PAYLOAD_LENGTH_MAX,
        "Remaining space wrong after PacketView initializing constructor");
    //bool ok = packet1.addField(mip::commands_base::Ping::FIELD_DESCRIPTOR, nullptr, 0);
    //check(ok, "Failed to add field to packet1");
    //check_equal(packet1.remainingSpace(), mip::C::MIP_PACKET_PAYLOAD_LENGTH_MAX-2, "remainingSpace wrong after adding Ping command to packet1");
    //check_equal(packet1.bufferSize(), sizeof(buffer), "Wrong buffer size after adding Ping command to packet1");
    packet1.finalize();

    std::ostringstream output;
    output << "Packet 1 not valid after finalization.\n";
    output << print_packet(packet1.pointer(), packet1.payloadLength());
    FAIL_AND_LOG_IF_NOT_TRUE(packet1.isValid(), output.str());

    mip::PacketView packet2(PING, sizeof(PING));
    checkPacketView(packet2, PING, "PacketView existing constructor");

    mip::C::mip_packet_view packet3c;
    mip::C::mip_packet_from_buffer(&packet3c, PING, sizeof(PING));
    mip::PacketView packet3(packet3c);
    checkPacketView(packet3, PING, "PacketView C constructor");

    mip::PacketView packet4(buffer, mip::commands_base::DESCRIPTOR_SET);
    checkPacketView(packet4, buffer, "PacketView initializing constructor (U8ArrayView version)");

    mip::PacketView packet5(PING);
    checkPacketView(packet5, PING, "PacketView existing constructor (U8ArrayView version)");
}

TEST("Packet buffer", "Packet buffer works as expected")
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

    // Construct from raw buffer, U8ArrayView, or existing view.
    mip::PacketBuf packet3(PING, sizeof(PING));
    mip::PacketBuf packet4(PING);
    mip::PacketBuf packet5((mip::PacketView(packet4)));

    checkPingPacketBuf(packet3, "PacketBuf raw buffer constructor");
    checkPingPacketBuf(packet4, "PacketBuf span constructor");
    checkPingPacketBuf(packet5, "PacketBuf PacketView constructor");

    // Regular copy constructor
    mip::PacketBuf packet6(packet5);
    checkPingPacketBuf(packet6, "PacketBuf copy constructor");
    check(packet6.bufferPointer() != packet5.bufferPointer(), "Packet6 shouldn't point to packet5's data buffer from copy constructor");

    // Construct from SizedPacketBuf of differing size
    mip::SizedPacketBuf<8> packet7(packet5);
    checkPingPacketView(packet7, "SizedPacketBuf<8> copy constructor");

    // Construction from PacketView
    mip::PacketBuf packet8(packet5.ref());
    checkPingPacketBuf(packet8, "PacketBuf copy constructor (PacketView)");
    check(packet8.bufferPointer() != packet5.bufferPointer(), "Packet6 shouldn't point to packet5's data buffer from copy constructor");

    // Assignment
    mip::PacketBuf packet9;
    packet9 = packet5;
    checkPingPacketBuf(packet9, "PacketBuf operator=");
    check(packet9.bufferPointer() != packet5.bufferPointer(), "Packet9 shouldn't point to packet5's data buffer from operator=");

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
    packet5.copyPacketTo({tmp, sizeof(tmp)});
    check(std::memcmp(PING, tmp, sizeof(PING))==0, "Temporary buffer doesn't match after calling packet.copyPacketTo (span version)");
}
