#include <mip/mip_packet.hpp>

#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/data_sensor.hpp>
#include <mip/definitions/data_shared.hpp>

#include <microstrain/platform.h>

#include <cstdio>

// This function demonstrates how to inspect and iterate fields in
// a MIP packet. These functions do not modify the packet and can
// be used on any packet as long as it has valid structure.
void print_packet(const mip::PacketView& packet, const char* name)
{
    // Ensure the packet structure is correct, otherwise a crash may result.
    // This checks the following:
    // 1. The packet buffer is not NULL.
    // 2. The buffer size is at least MIP_PACKET_LENGTH_MIN.
    // 3. The payload length does not exceed the buffer size.
    // There's generally no need to check this if you're getting packets
    // directly from mip_packet_create (with sufficient buffer space) or
    // from the mip parser. If you're reading packets from a file, etc.,
    // without parsing, use this as a cheap validation step.
    // In this example, it should always pass.
    assert(packet.isSane());

    // Roughly equivalent to packet.checksumValue() == packet.computeChecksum()
    const char* checksum_str = packet.isValid() ? "valid" : "invalid";

    // Print descriptor set, total length, and payload length.
    std::printf(
        "%s: desc_set=0x%02X tot_len=%u pay_len=%u chk=%s [",
        name, packet.descriptorSet(), packet.totalLength(), packet.payloadLength(), checksum_str
    );

    // Print each byte in the packet, including header and checksum.
    for(size_t i=0; i < packet.totalLength(); i++)
    {
        std::printf("%02X", packet[i]);
    }

    std::puts("]");

    // Iterate each field in the packet and print it.
    for(mip::FieldView field : packet)
    {
        // Print descriptors (descriptor set always matches the packet).
        std::printf("    (%02X,%02X): payload=[", field.descriptorSet(), field.fieldDescriptor());

        // Print field payload bytes.
        for(size_t i=0; i < field.payloadLength(); i++)
        {
            // Note: this indexes the payload, excluding the header, whereas packet[i] indexes the whole packet.
            std::printf("%02X", field[i]);
        }

        std::puts("]");
    }
}

// This function demonstrates how to build mip packets from scratch.
// It starts with simple cases and moves to more complex ones. It
// also shows some of the low-level details to help you understand
// what's happening "under the hood".
void create_packet_from_scratch()
{
    std::puts("\nCreate packet from scratch");

    // Create a mip packet with a built-in buffer.
    // A PacketBuf is the combination of a PacketView and a buffer.
    //
    // If the descriptor set is specified (even if it's invalid, i.e. 0x00)
    // the constructor initializes a new MIP packet in the buffer.
    // Otherwise, it assumes the buffer already contains a valid MIP packet.

#if 1  // These sections are identical
    mip::PacketBuf packet(mip::commands_base::DESCRIPTOR_SET);
#else
    uint8_t buffer[mip::PACKET_LENGTH_MAX];
    mip::PacketView packet(buffer, sizeof(buffer), mip::commands_base::DESCRIPTOR_SET);
#endif

    // The packet isn't ready yet, but print it to demonstrate what the raw bytes look like after each step.
    print_packet(packet, "Empty, un-finalized packet");

    // Write the checksum.
    packet.finalize();
    print_packet(packet, "Empty packet with checksum");

    //
    // FIELD #1
    //
    // A Ping command, with no payload data.
    //

    packet.addField(mip::commands_base::CMD_PING, nullptr, 0);

    //
    // FIELD #2
    //
    // A Base Comm Speed command using a fixed/pre-serialized payload.
    //
    // If you already have the payload data in a buffer, this is the
    // most efficient way to add a field.
    //

    const uint8_t payload2[] = { 0x01, 0x01, 0x00, 0x01, 0xC2, 0x00 };
    packet.addField(mip::commands_base::CMD_COMM_SPEED, payload2, sizeof(payload2));

    print_packet(packet, "Unfinished packet with 2 fields");

    //
    // FIELD #3
    //
    // A Base Comm Speed command, using the C++ struct definition.
    //
    // Generally, the C++ struct definitions are the easiest and most readable way to
    // create a field in a packet from its parameter values. It's also the most efficient
    // method offered by the MIP SDK because the data is not copied through any intermediate
    // buffers; it's serialized directly from the command struct to the packet.
    //

    mip::commands_base::CommSpeed cmd3;
    cmd3.function = mip::FunctionSelector::WRITE;
    cmd3.port     = 0x01;
    cmd3.baud     = 115200;

    // In C++, the command can be directly added via a template function which takes
    // care of the boilerplate of allocating and serializing a field. Compare to the
    // C version of field #3 in mip_packet_example.c.
    packet.addField(cmd3);

    print_packet(packet, "Unfinished packet with 3 fields");

    //
    // FIELD #4
    //
    // A Base Comm Speed command, by manually serializing the parameters.
    //
    // This is exactly the same as field #3, but without the helper
    // functions. This is intended to show what happens "behind the scenes".
    //

    mip::Serializer serializer4 = packet.createField(mip::commands_base::CommSpeed::FIELD_DESCRIPTOR, 6);

#if 1
    // insert is a variadic template allowing serialization of multiple values at once.
    // This can offer efficiency improvements over the C version in some cases by bypassing some overhead.
    serializer4.insert(
        uint8_t(0x01),
        uint8_t(0x01),
        uint32_t(115200)
    );
#else  // Effectively the same, but possibly less efficient.
    serialize4.insert<uint8_t>(0x01);
    serialize4.insert<uint8_t>(0x01);
    serialize4.insert<uint32_t>(115200);
#endif

    //
    // Complete packet #1
    //

    // Update the checksum.
    packet.finalize();

    // "Send" the packet
    print_packet(packet, "Finished packet with 4 fields");

    //
    // Packet #2
    //

    // Start over with a new descriptor set.
    packet.reset(mip::commands_3dm::DESCRIPTOR_SET);

    // Payload length has been zeroed out. Checksum is garbage.
    print_packet(packet, "Empty packet after reset");

    //
    // FIELD #5 (second packet)
    //
    // A 3DM Message Format command field using the C struct definition.
    //
    // Similar to #3 except that we have a variable-length payload.
    // Again, this is the recommended method for field creation.
    //

    mip::commands_3dm::MessageFormat cmd5;
    cmd5.function = mip::FunctionSelector::WRITE;
    cmd5.desc_set = mip::data_sensor::DESCRIPTOR_SET;
    cmd5.num_descriptors = 3;  // This determines how many descriptors follow.
    cmd5.descriptors[0].descriptor = mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR;
    cmd5.descriptors[1].descriptor = mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR;
    cmd5.descriptors[2].descriptor = mip::data_sensor::DATA_GYRO_SCALED;
    cmd5.descriptors[0].decimation = 10;
    cmd5.descriptors[1].decimation = 10;
    cmd5.descriptors[2].decimation = 10;

    packet.addField(cmd5);

    packet.finalize();
    print_packet(packet, "3DM Message Format command");
    mip_packet_reset(&packet, mip::commands_3dm::DESCRIPTOR_SET);

    //
    // Packet #3
    //

    // FIELD #6 (third packet)
    //
    // A 3DM Message Format command field, by manually serializing parameters
    //
    // Similar to #4 except that with a variable-length payload.
    // Similar to #5, but with lower level function calls and slightly different command.
    //

    // Create a field of unknown length.
    // This is equivalent to creating a field with length 0 and a serializer covering
    // all of the remaining space. After the field is serialized, the field length is
    // updated based on how many bytes were used.
    mip::PacketView::AllocatedField field6 = packet.createField(mip::commands_3dm::PollData::FIELD_DESCRIPTOR);

    field6.insert(
        false, // suppress_ack
        uint8_t(mip::data_sensor::DESCRIPTOR_SET),
        uint8_t(3)
    );

    // Arrays can also be directly serialized if their size is known.
#if 1
    std::array<uint8_t, 3> descriptors6 = {{
        mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR,
        mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR,
        mip::data_sensor::DATA_GYRO_SCALED,
    }};
    field6.insert(descriptors6);
#else
    uint8_t descriptors6[] = {
        mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR,
        mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR,
        mip::data_sensor::DATA_GYRO_SCALED,
    };
    field6.insert(descriptors6);
    // Equivalent to
    //field6.insert(descriptors6, 3);
#endif

    // Update the field length.
    // Note that if the field would exceed the remaining space in the packet, this will
    // instead remove the field entirely and return false.
    bool ok6 = field6.commit();
    assert(ok6);  // Shouldn't happen in this example.

    //
    // Complete packet #3
    //

    mip_packet_finalize(&packet);
    print_packet(&packet, "3DM Poll Data command");
}

void create_packet_from_buffer()
{
    std::puts("\nCreate packet from buffer");

    const uint8_t buffer[] = {
        0x75, 0x65, 0x80, 0x4c, 0x0a, 0xd5, 0x00, 0x00, 0x00, 0x05, 0x5e, 0xe6, 0x7c, 0xc0, 0x0a, 0xd6,
        0x00, 0x00, 0x00, 0x01, 0x4e, 0x43, 0x4a, 0x00, 0x0e, 0x04, 0x3d, 0x9e, 0xe8, 0x8d, 0x38, 0x7f,
        0xdb, 0x00, 0xbf, 0x7a, 0xaf, 0x03, 0x0e, 0x05, 0xbb, 0x0c, 0x1e, 0x30, 0xbb, 0x57, 0x2e, 0x68,
        0xbb, 0xaa, 0x24, 0xae, 0x0e, 0x07, 0xbc, 0x8a, 0xac, 0x80, 0xbc, 0x72, 0xc5, 0x0e, 0xbc, 0xc4,
        0xe2, 0xc1, 0x0e, 0x08, 0x3e, 0xee, 0x3d, 0x9f, 0xbd, 0x66, 0xda, 0xdd, 0xc0, 0xaf, 0xde, 0xf5,
        0x91, 0x96
    };

    // Create a view of the packet in the buffer.
    // Note: The buffer must not be modified, so do not call functions that manipulate the packet.
    // E.g. finalize(), addField, createField, reset, etc.
    mip::PacketView packet1(buffer, sizeof(buffer));

    // Ensure the packet is valid before inspecting it.
    // This is the combination of checking:
    // 1. mip_packet_is_sane (buffer size and payload length checks)
    // 2. The packet has a non-zero descriptor set.
    // 3. The checksum is valid.
    if(!packet1.isValid())
    {
        assert(false); // The packet should be valid in this example.
        return;
    }

    // Print the packet.
    print_packet(packet1, "Packet from buffer");

    // Create a view from a span.
    microstrain::Span<const uint8_t> span(buffer, sizeof(buffer));
    mip::PacketView packet2(span);
    print_packet(packet2, "Packet from span");

    // Create a C++ PacketView from the C equivalent.
    mip::C::mip_packet_view packet3c;
    mip::C::mip_packet_from_buffer(&packet3c, buffer, sizeof(buffer));
    mip::PacketView packet3(packet3c);
    print_packet(packet3, "Packet from C packet");

    // Example of what an application might do to parse specific data.
    // This example is for demonstration of the techniques used with the MIP SDK;
    // consider using the "dispatch" system (see the documentation) instead.

    // Is this a sensor data packet?
    if(packet1.descriptorSet() == mip::data_sensor::DESCRIPTOR_SET)
    {
        std::puts("Sensor Data packet:");

        for(mip::FieldView field : packet1)
        {
            mip::Serializer serializer(field.payloadSpan());

            switch(field.fieldDescriptor())
            {
            case mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR:
            {
                uint64_t nanoseconds;

                if(serializer.extract(nanoseconds))
#if defined MICROSTRAIN_PLATFORM_APPLE
                    std::printf("  Ref Time = %llu\n", nanoseconds);
#else
                    std::printf("  Ref Time = %lu\n", nanoseconds);
#endif // MICROSTRAIN_PLATFORM_APPLE

                break;
            }
            case mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR:
            {
                mip::data_sensor::ScaledAccel data;

                if(serializer.extract(data))
                    std::printf("  Scaled Accel = (%f, %f, %f)\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2]);

                break;
            }
            case mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR:
            {
                mip::data_sensor::ScaledGyro data;

                // This calls the FieldView::extract helper function which handles deserialization directly.
                // This is the simplest and preferred method.
                if(field.extract(data))
                    std::printf("  Scaled Gyro = (%f, %f, %f)\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);

                break;
            }
            }
        }
    }
}

int main()
{
    create_packet_from_scratch();
    create_packet_from_buffer();
}
