#include "mip/mip_packet.h"
#include "mip/mip_serialization.h"

#include "mip/definitions/commands_3dm.h"
#include "mip/definitions/commands_base.h"
#include "mip/definitions/data_sensor.h"
#include "mip/definitions/data_shared.h"

#include <microstrain/platform.h>

#include <stdbool.h>
#include <stdio.h>


// This function demonstrates how to inspect and iterate fields in
// a MIP packet. These functions do not modify the packet and can
// be used on any packet as long as it has valid structure.
void print_packet(const mip_packet_view* packet, const char* name)
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
    assert(mip_packet_is_sane(packet));

    // Roughly equivalent to mip_packet_checksum_value(packet) == mip_packet_compute_checksum(packet).
    const char* checksum_str = mip_packet_is_valid(packet) ? "valid" : "invalid";

    // Print descriptor set, total length, and payload length.
    printf(
        "%s: desc_set=0x%02X tot_len=%u pay_len=%u chk=%s [",
        name, mip_packet_descriptor_set(packet), mip_packet_total_length(packet), mip_packet_payload_length(packet), checksum_str
    );

    // Print each byte in the packet, including header and checksum.
    for(size_t i = 0; i < mip_packet_total_length(packet); i++)
    {
        printf("%02X", mip_packet_pointer(packet)[i]);
    }

    puts("]");

    // Iterate each field in the packet and print it.
    mip_field_view field;
    mip_field_init_empty(&field);
    while(mip_field_next_in_packet(&field, packet))
    {
        // Print descriptors (descriptor set always matches the packet).
        printf("    (%02X,%02X): payload=[", mip_field_descriptor_set(&field), mip_field_field_descriptor(&field));

        // Print field payload bytes.
        for(size_t i = 0; i < mip_field_payload_length(&field); i++)
        {
            printf("%02X", mip_field_payload(&field)[i]);
        }

        puts("]");
    }
}

// This function demonstrates how to build mip packets from scratch.
// It starts with simple cases and moves to more complex ones. It
// also shows some of the low-level details to help you understand
// what's happening "under the hood".
void create_packet_from_scratch()
{
    puts("\nCreate packet from scratch");

    // Create a mip packet and storage buffer.
    mip_packet_view packet;
    uint8_t buffer[MIP_PACKET_LENGTH_MAX];
    mip_serializer serializer;

    // If the descriptor set is specified (even if it's invalid, i.e. 0x00)
    // the constructor initializes a new MIP packet in the buffer.
    // Otherwise, it assumes the buffer already contains a valid MIP packet.
    mip_packet_create(&packet, buffer, sizeof(buffer), MIP_BASE_CMD_DESC_SET);

    // The packet isn't ready yet, but print it to demonstrate what the raw bytes look like after each step.
    print_packet(&packet, "Empty, un-finalized packet");

    // Write the checksum.
    mip_packet_finalize(&packet);
    print_packet(&packet, "Empty packet with checksum");

    //
    // FIELD #1
    //
    // A Ping command, with no payload data.
    //

    mip_packet_add_field(&packet, MIP_CMD_DESC_BASE_PING, NULL, 0);

    //
    // FIELD #2
    //
    // A Base Comm Speed command using a fixed/pre-serialized payload.
    //
    // If you already have the payload data in a buffer, this is the
    // most efficient way to add a field.
    //

    const uint8_t payload2[] = { 0x01, 0x01, 0x00, 0x01, 0xC2, 0x00 };
    mip_packet_add_field(&packet, MIP_CMD_DESC_BASE_COMM_SPEED, payload2, sizeof(payload2));

    print_packet(&packet, "Unfinished packet with 2 fields");

    //
    // FIELD #3
    //
    // A Base Comm Speed command, using the C struct definition.
    //
    // Generally, the C struct definitions are the easiest and most readable way to
    // create a field in a packet from its parameter values. It's also the most efficient
    // method offered by the MIP SDK because the data is not copied through any intermediate
    // buffers; it's serialized directly from the command struct to the packet.
    //

    mip_base_comm_speed_command cmd3;
    cmd3.function = MIP_FUNCTION_WRITE;
    cmd3.port     = 0x01;
    cmd3.baud     = 115200;

    // Create a new field with a serializer.
    mip_serializer_init_new_field(&serializer, &packet, MIP_CMD_DESC_BASE_COMM_SPEED);
    // Serialize the command directly into the field's payload.
    insert_mip_base_comm_speed_command(&serializer, &cmd3);
    // Update the field length byte.
    mip_serializer_finish_new_field(&serializer, &packet);

    print_packet(&packet, "Unfinished packet with 3 fields");

    //
    // FIELD #4
    //
    // A Base Comm Speed command, by manually serializing the parameters.
    //
    // This is exactly the same as field #3, but without the mip_serializer_*_field helper
    // functions. This is intended to show what happens "behind the scenes".
    //

    // This part is analogous to mip_serializer_init_new_field from field #3.
    // --------
    uint8_t* payload4 = NULL;
    // Create a field and obtain the payload pointer.
    // The return value is the number of bytes remaining after allocating this field.
    // Note that in this particular case, we know the field length will be 6 bytes.
    // This allows us to save going back and updating the field length after serialization (see field #6).
    int leftover3 = mip_packet_create_field(&packet, MIP_CMD_DESC_BASE_COMM_SPEED, 6, &payload4);

    // If less than 0 bytes are left over, then allocation failed by this many bytes.
    assert(leftover3 >= 0 && payload4 != NULL);

    // Show what the packet looks like after allocating a field.
    // The header will exist but the payload will not be valid yet.
    print_packet(&packet, "Field #4 allocated, not written");

    // Initialize the serializer for the payload.
    microstrain_serializer_init_insertion(&serializer, payload4, 6);
    // --------

    // This part is analogous to insert_mip_base_comm_speed_command
    // --------
    // Write parameters to the payload.
    microstrain_insert_u8(&serializer, 0x01);
    microstrain_insert_u8(&serializer, 0x01);
    microstrain_insert_u32(&serializer, 115200);
    // Make sure serialization was successful (not out of space).
    assert(microstrain_serializer_is_ok(&serializer));
    // --------

    // The equivalent to mip_serializer_finish_new_field isn't required in this case
    // because we specified an exact payload length to mip_packet_create_field.


    //
    // Complete packet #1
    //

    // Update the checksum.
    mip_packet_finalize(&packet);

    // "Send" the packet
    print_packet(&packet, "Finished packet with 4 fields");

    //
    // Packet #2
    //

    // Start over with a new descriptor set.
    mip_packet_reset(&packet, MIP_3DM_CMD_DESC_SET);

    // Payload length has been zeroed out. Checksum is garbage.
    print_packet(&packet, "Empty packet after reset");

    //
    // FIELD #5 (second packet)
    //
    // A 3DM Message Format command field using the C struct definition.
    //
    // Similar to #3 except that we have a variable-length payload.
    // Again, this is the recommended method for field creation.
    //

    mip_3dm_message_format_command cmd5;
    cmd5.function = MIP_FUNCTION_WRITE;
    cmd5.desc_set = MIP_SENSOR_DATA_DESC_SET;
    cmd5.num_descriptors = 3;  // This determines how many descriptors follow.
    cmd5.descriptors[0].descriptor = MIP_DATA_DESC_SHARED_REFERENCE_TIME;
    cmd5.descriptors[1].descriptor = MIP_DATA_DESC_SENSOR_ACCEL_SCALED;
    cmd5.descriptors[2].descriptor = MIP_DATA_DESC_SENSOR_GYRO_SCALED;
    cmd5.descriptors[0].decimation = 10;
    cmd5.descriptors[1].decimation = 10;
    cmd5.descriptors[2].decimation = 10;

    // Create the field and init the serializer in one step.
    mip_serializer_init_new_field(&serializer, &packet, MIP_CMD_DESC_3DM_MESSAGE_FORMAT);
    // Serialize the command.
    insert_mip_3dm_message_format_command(&serializer, &cmd5);
    // Update the field length.
    mip_serializer_finish_new_field(&serializer, &packet);

    mip_packet_finalize(&packet);
    print_packet(&packet, "3DM Message Format command");
    mip_packet_reset(&packet, MIP_3DM_CMD_DESC_SET);

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

    // This part is analogous to mip_serializer_init_new_field from field #3.
    // It's almost the same as in field #4, except we use a zero-length field.
    // --------
    // Get a pointer to the payload buffer and the maximum number of bytes that can fit.
    //
    // Start by creating a zero-payload-length field and determining the remaining space.
    uint8_t* payload5 = NULL;
    int available5 = mip_packet_create_field(&packet, MIP_CMD_DESC_3DM_POLL_DATA, 0, &payload5);
    // If less than 0 bytes are available, then there wasn't even space for the field header.
    assert(available5 > 0 && payload5 != NULL);
    // Initialize the serializer, giving it all available space.
    microstrain_serializer_init_insertion(&serializer, payload5, available5);
    // --------

    // This part is analogous to insert_mip_3dm_poll_command
    // --------
    // Build a 3DM Poll Data command:
    // 1. Suppress_ack
    microstrain_insert_bool(&serializer, false);
    // 2. Descriptor set
    microstrain_insert_u8(&serializer, MIP_SENSOR_DATA_DESC_SET);
    // 3. Number of data quantities.
    unsigned int num_data = 3;  // Configure two descriptors
    microstrain_insert_u8(&serializer, num_data);
    // 4. Array of uint8_t descriptors.
    // Descriptor 1 - Reference timestamp
    microstrain_insert_u8(&serializer, MIP_DATA_DESC_SHARED_REFERENCE_TIME);
    // Descriptor 2 - Scaled Accel
    microstrain_insert_u8(&serializer, MIP_DATA_DESC_SENSOR_ACCEL_SCALED);
    // Descriptor 3 - Scaled Gyro
    microstrain_insert_u8(&serializer, MIP_DATA_DESC_SENSOR_GYRO_SCALED);
    // --------

    // This part is analogous to mip_serializer_finish_new_field
    // --------
    // Check that everything fit in the field payload.
    if(microstrain_serializer_is_ok(&serializer))
    {
        // Now we know how many bytes were written, so update the field length in the packet.
        mip_packet_update_last_field_length(&packet, payload5, (uint8_t)microstrain_serializer_length(&serializer));
    }
    else // Not enough space
    {
        assert(false); // This shouldn't happen in this example (note, there's no assertion in mip_serializer_finish_new_field).
        // Cancel the field to ensure the packet is still valid.
        mip_packet_cancel_last_field(&packet, payload5);
    }
    // --------

    //
    // Complete packet #3
    //

    mip_packet_finalize(&packet);
    print_packet(&packet, "3DM Poll Data command");
}

// This function demonstrates how to construct a packet view from a buffer with existing data.
// For simplicity, the packet is hardcoded.
void create_packet_from_buffer()
{
    puts("\nCreate packet from buffer");

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
    mip_packet_view packet1;
    mip_packet_from_buffer(&packet1, buffer, sizeof(buffer));

    // Ensure the packet is valid before inspecting it.
    // This is the combination of checking:
    // 1. mip_packet_is_sane (buffer size and payload length checks)
    // 2. The packet has a non-zero descriptor set.
    // 3. The checksum is valid.
    if(!mip_packet_is_valid(&packet1))
    {
        assert(false); // The packet should be valid in this example.
        return;
    }

    // Print the packet.
    print_packet(&packet1, "Packet from buffer");

    // Example of what an application might do to parse specific data.
    // This example is for demonstration of the techniques used with the MIP SDK;
    // consider using the "dispatch" system (see the documentation) instead.

    // Is this a sensor data packet?
    if(mip_packet_descriptor_set(&packet1) == MIP_SENSOR_DATA_DESC_SET)
    {
        puts("Sensor Data packet:");

        // Iterate all fields in the packet.
        mip_field_view field;
        mip_field_init_empty(&field);
        while(mip_field_next_in_packet(&field, &packet1))
        {
            // Create a deserializer for the field.
            mip_serializer serializer;
            microstrain_serializer_init_extraction(&serializer, mip_field_payload(&field), mip_field_payload_length(&field));

            // Check what data the field contains.
            switch(mip_field_field_descriptor(&field))
            {
            case MIP_DATA_DESC_SHARED_REFERENCE_TIME:
            {
                uint64_t nanoseconds;
                microstrain_extract_u64(&serializer, &nanoseconds);

                if(microstrain_serializer_is_complete(&serializer))
#if defined MICROSTRAIN_PLATFORM_APPLE
                    printf("  Ref Time = %llu\n", nanoseconds);
#else
                    printf("  Ref Time = %lu\n", nanoseconds);
#endif // MICROSTRAIN_PLATFORM_APPLE

                break;
            }
            case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
            {
                // Extract the 3-vector.
                float x,y,z;
                microstrain_extract_float(&serializer, &x);
                microstrain_extract_float(&serializer, &y);
                microstrain_extract_float(&serializer, &z);

                // Check if the entire field was deserialized (validity check).
                if(microstrain_serializer_is_complete(&serializer))
                    printf("  Scaled Accel = (%f, %f, %f)\n", x, y, z);

                break;
            }
            case MIP_DATA_DESC_SENSOR_GYRO_SCALED:
            {
                // Same as scaled accel except using the C data structure.
                // This is the recommended method.
                mip_sensor_scaled_gyro_data data;
                extract_mip_sensor_scaled_gyro_data(&serializer, &data);

                if(microstrain_serializer_is_complete(&serializer))
                    printf("  Scaled Gyro = (%f, %f, %f)\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);

                break;
            }
            }
        }
        puts("");
    }
}

int main()
{
    create_packet_from_scratch();
    create_packet_from_buffer();

    return 0;
}
