////////////////////////////////////////////////////////////////////////////////
/// mip_packet_example.c
///
/// Example program to create raw and buffered MIP packets using C
///
/// For this example, we have broken down each piece into separate functions
/// for easier documentation and is not necessary in practice.
///
/// If this example does not meet your specific setup needs, please consult the
/// MIP SDK API documentation for the proper commands.
///
/// @section LICENSE
///
/// THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
/// WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
/// TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD LIABLE FOR ANY
/// DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
/// FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
/// CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
///

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/mip_packet.h>
#include <mip/mip_serialization.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/data_sensor.h>
#include <mip/definitions/data_shared.h>

#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// Print the state of a packet
void print_packet(const mip_packet_view* _packet_view);

// Initialize an empty packet for a given descriptor set
void initialize_empty_packet(mip_packet_view* _packet_view, uint8_t* _buffer, const size_t _buffer_size,
    const uint8_t _descriptor_set);

// Compute and add the checksum to a packet
void add_checksum_to_packet(mip_packet_view* _packet_view);

// Fields added to packet 1
void add_ping_command_to_packet(mip_packet_view* _packet_view);
void add_comm_speed_bytes_to_packet(mip_packet_view* _packet_view);
void add_comm_speed_field_to_packet(mip_packet_view* _packet_view);
void add_comm_speed_serializer_bytes_to_packet(mip_packet_view* _packet_view);

// Fields added to packet 2
void add_message_format_field_to_packet(mip_packet_view* _packet_view);

// Fields added to packet 3
void add_poll_data_field_to_packet(mip_packet_view* _packet_view);

// Fields extracted from packet 4
void extract_shared_reference_time_field(microstrain_serializer* _serializer);
void extract_shared_reference_time_delta_field(microstrain_serializer* _serializer);
void extract_sensor_accel_scaled_field(microstrain_serializer* _serializer);
void extract_sensor_gyro_scaled_field(microstrain_serializer* _serializer);
void extract_sensor_delta_theta_field(microstrain_serializer* _serializer);
void extract_sensor_delta_velocity_field(const mip_field_view* _field_view);

// Packet creation
void create_packet_1_from_scratch();
void create_packet_2_and_3_from_scratch();
void create_packet_4_from_raw_buffer();

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Create packet 1 with multiple fields
    create_packet_1_from_scratch();

    // Create packet 2 with a single field, then reset the packet and create packet 3 with a single field
    create_packet_2_and_3_from_scratch();

    // Create packet 4 with a raw buffer and extract each field from the packet
    create_packet_4_from_raw_buffer();

    printf("Example Completed Successfully.\n");

#ifdef _WIN32
    // Keep the console open on Windows
    system("pause");
#endif // _WIN32

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Prints detailed information about a MIP packet's structure and
///        contents
///
/// @details This function inspects and displays:
///          - Total packet length
///          - Raw packet bytes in hex format
///          - MIP SYNC bytes
///          - Descriptor set
///          - Payload length
///          - Field information for each field in the packet
///          - Checksum values and validity
///
/// @param _packet_view Pointer to the MIP packet view to inspect
///
void print_packet(const mip_packet_view* _packet_view)
{
    printf("Packet information:\n");

    // Ensure the packet structure is correct, otherwise a crash may result
    // This checks the following:
    //     1. The packet buffer is not NULL
    //     2. The buffer size is at least MIP_PACKET_LENGTH_MIN
    //     3. The payload length does not exceed the buffer size
    // There's generally no need to check this if you're getting packets
    // directly from mip_packet_create (with sufficient buffer space) or
    // from the mip parser
    // If you're reading packets from a file, etc., without parsing, use
    // this as an inexpensive validation step
    assert(mip_packet_is_sane(_packet_view));

    const uint8_t* packet_pointer = mip_packet_pointer(_packet_view);

    // Create a buffer for printing purposes
    char packet_byte_buffer[MIP_PACKET_PAYLOAD_LENGTH_MAX] = { 0 };
    int  buffer_offset                                     = 0;

    // Get each byte in the packet, including header and checksum
    for (size_t i = 0; i < mip_packet_total_length(_packet_view); i++)
    {
        buffer_offset += snprintf(
            &packet_byte_buffer[buffer_offset],
            sizeof(packet_byte_buffer) / sizeof(packet_byte_buffer[0]) - buffer_offset,
            "%02X",
            packet_pointer[i]
        );
    }

    const uint8_t payload_length = mip_packet_payload_length(_packet_view);

    // Print the packet details before the fields
    printf("%4s%-20s = %u\n", " ", "Total Length", mip_packet_total_length(_packet_view));
    printf("%4s%-20s = %s\n", " ", "Raw Packet", packet_byte_buffer);
    printf("%4s%-20s = 0x%02X\n", " ", "MIP SYNC1", packet_pointer[0]);
    printf("%4s%-20s = 0x%02X\n", " ", "MIP SYNC2", packet_pointer[1]);
    printf("%4s%-20s = 0x%02X\n", " ", "Descriptor Set", mip_packet_descriptor_set(_packet_view));
    printf("%4s%-20s = 0x%02X\n", " ", "Payload Length", payload_length);

    // Check that fields exist using the payload length
    if (payload_length > 0)
    {
        printf("%4sFields:", " ");
    }

    // Field object for iterating the packet and extracting each field
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    // Iterate the packet and extract each field
    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        printf("\n");

        // Print descriptors (the descriptor set always matches the packet)
        // Include the size of the length byte
        printf("%8s%-16s = 0x%02X\n", " ", "Field Length", mip_field_payload_length(&field_view) + 2);
        printf("%8s%-16s = 0x%02X\n", " ", "Field Descriptor", mip_field_field_descriptor(&field_view));
        printf("%8s%-16s = ", " ", "Raw Payload");

        // Print field payload bytes.
        for (size_t i = 0; i < mip_field_payload_length(&field_view); i++)
        {
            printf("%02X", mip_field_payload(&field_view)[i]);
        }

        printf("\n");
    }

    // Print the checksum most and least significant bytes, and if it's valid or not
    const uint16_t checksum_value = mip_packet_checksum_value(_packet_view);
    printf("%4sChecksum (%s):\n", " ", mip_packet_is_valid(_packet_view) ? "Valid" : "Invalid");
    printf("%8s%-16s = 0x%02X\n", " ", "MSB", checksum_value >> 0x08);
    printf("%8s%-16s = 0x%02X\n\n", " ", "LSB", checksum_value &  0xFF);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes an empty MIP packet with the specified descriptor set
///
/// @details Creates a new MIP packet in the provided buffer and initializes it
///          with the given descriptor set. The packet is initially empty and
///          invalid until fields are added.
///
/// @param _packet_view Pointer to packet view to initialize
/// @param _buffer Buffer to store the packet data
/// @param _buffer_size Size of the buffer in bytes
/// @param _descriptor_set Descriptor set to use for the packet
///
void initialize_empty_packet(mip_packet_view* _packet_view, uint8_t* _buffer, const size_t _buffer_size,
    const uint8_t _descriptor_set)
{
    // If the descriptor set is specified, even if it's invalid, i.e., 0x00,
    // the constructor initializes a new MIP packet in the buffer.
    // Otherwise, it assumes the buffer already contains a valid MIP packet.
    mip_packet_create(_packet_view, _buffer, _buffer_size, _descriptor_set);

    printf("Created an empty packet for descriptor set 0x%02X.\n", _descriptor_set);

    // Print the current state of the packet
    // Note: The packet is currently empty and invalid
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Computes and adds a checksum to a MIP packet
///
/// @details Finalizes the packet by computing and appending the checksum.
///          This should be called after all fields have been added to the
///          packet.
///
/// @param _packet_view Pointer to the packet to finalize with checksum
///
void add_checksum_to_packet(mip_packet_view* _packet_view)
{
    mip_packet_finalize(_packet_view);

    printf("Added a checksum to the packet.\n");

    // Print the current state of the packet
    // Note: The packet is now validated
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Ping command field to a MIP packet
///
/// @details Creates a field with the Base Ping command descriptor (0x01) with
///          no payload data.
///
/// @remark Field 1
///
/// @param _packet_view Pointer to the packet to add the field to
///
void add_ping_command_to_packet(mip_packet_view* _packet_view)
{
    mip_packet_add_field(
        _packet_view,
        MIP_CMD_DESC_BASE_PING, // Field descriptor set
        NULL,                   // Payload data
        0                       // Payload buffer length
    );

    printf("Added a Ping command field to the packet.\n");

    // Print the current state of the packet
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Comm Speed command field using raw bytes
///
/// @details Demonstrates adding a field using pre-serialized payload data.
///          This is the most efficient method when payload data is already
///          available.
///
/// @remark Field 2
///
/// @param _packet_view Pointer to the packet to add the field to
///
void add_comm_speed_bytes_to_packet(mip_packet_view* _packet_view)
{
    // Build the raw payload for the packet
    const uint8_t comm_speed_payload[] = {
        0x01,                  // Function selector
        0x01,                  // Port
        0x00, 0x01, 0xC2, 0x00 // Baudrate
    };

    mip_packet_add_field(
        _packet_view,
        MIP_CMD_DESC_BASE_COMM_SPEED,                              // Field descriptor set
        comm_speed_payload,                                        // Payload data
        sizeof(comm_speed_payload) / sizeof(comm_speed_payload[0]) // Payload buffer length
    );

    printf("Added a Comm Speed command field to the packet using raw bytes.\n");

    // Print the current state of the packet
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Comm Speed command field using a field struct
///
/// @details Shows how to add a field using the field struct definitions, which
///          is the recommended approach for creating fields from parameter
///          values.
///
/// @remark Field 3
///
/// @param _packet_view Pointer to the packet to add the field to
///
void add_comm_speed_field_to_packet(mip_packet_view* _packet_view)
{
    mip_base_comm_speed_command comm_speed_command;
    comm_speed_command.function = MIP_FUNCTION_WRITE;
    comm_speed_command.port     = 0x01;
    comm_speed_command.baud     = 115200;

    // Create a new field with a serializer.
    mip_serializer serializer;
    mip_serializer_init_new_field(&serializer, _packet_view, MIP_CMD_DESC_BASE_COMM_SPEED);

    // Serialize the command directly into the field's payload.
    insert_mip_base_comm_speed_command(&serializer, &comm_speed_command);

    // Update the field length byte.
    mip_serializer_finish_new_field(&serializer, _packet_view);

    printf("Added a Comm Speed command field to the packet using a field.\n");

    // Print the current state of the packet
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Comm Speed command field using manual serialization
///
/// @details Demonstrates low-level field creation by manually serializing
///          parameters. Shows the underlying process that happens with
///          struct-based methods.
///
/// @remark Field 4
///
/// @note This is exactly the same as field 3 but without the
///       mip_serializer_*_field helper functions.
///       This is intended to show what happens "behind the scenes".
///
/// @param _packet_view Pointer to the packet to add the field to
///
void add_comm_speed_serializer_bytes_to_packet(mip_packet_view* _packet_view)
{
    // This part is analogous to 'mip_serializer_init_new_field' from field 3
    uint8_t*      payload        = NULL;
    const uint8_t payload_length = 6;

    // Create a field and get the payload pointer
    // The return value is the number of bytes remaining after allocating this field
    // Note: We know the field length will be 6 bytes, allowing us to not have to update
    // the field length after initialization as field 6 does
    const int remaining_bytes = mip_packet_create_field(
        _packet_view,
        MIP_CMD_DESC_BASE_COMM_SPEED,
        payload_length,
        &payload
    );
    (void)remaining_bytes; // Remaining size of the packet (unused value)

    // If less than 0 bytes are left over, then allocation failed by this many bytes
    assert(remaining_bytes >= 0 && payload != NULL);

    printf("Reserved space in the packet for a Comm Speed command field.\n");

    // Show what the packet looks like after allocating a field
    // The header will exist, but the payload will not be valid yet
    print_packet(_packet_view);

    // Initialize the serializer for the payload
    microstrain_serializer serializer;
    microstrain_serializer_init_insertion(&serializer, payload, payload_length);

    // Write parameters to the payload
    // Note: This is analogous to insert_mip_base_comm_speed_command
    microstrain_insert_u8(&serializer, MIP_FUNCTION_WRITE);
    microstrain_insert_u8(&serializer, 0x01);
    microstrain_insert_u32(&serializer, 115200);

    // Make sure the serializer is not out of space
    assert(microstrain_serializer_is_ok(&serializer));

    // Note: The equivalent to mip_serializer_finish_new_field isn't required in this case
    // because we specified an exact payload length to mip_packet_create_field

    printf("Added a Comm Speed command field to the packet using a serializer.\n");

    // Print the current state of the packet
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Message Format command field using a field struct definition
///
/// @details Creates a field for configuring the message format with multiple
///          descriptors, showing how to handle variable-length payloads.
///
/// @remark Field 5
///
/// @note Similar to field 3 except that we have a variable-length payload.
///       Again, this is the recommended method for field creation.
///
/// @param _packet_view Pointer to the packet to add the field to
///
void add_message_format_field_to_packet(mip_packet_view* _packet_view)
{
    mip_3dm_message_format_command message_format;

    // Mark the command for writing
    message_format.function = MIP_FUNCTION_WRITE;

    // Set the message format data descriptor set
    message_format.desc_set = MIP_SENSOR_DATA_DESC_SET;

    // Number of descriptors to include
    message_format.num_descriptors = 3;

    // First descriptor to include
    message_format.descriptors[0].descriptor = MIP_DATA_DESC_SHARED_REFERENCE_TIME;
    message_format.descriptors[0].decimation = 10;

    // Second descriptor to include
    message_format.descriptors[1].descriptor = MIP_DATA_DESC_SENSOR_ACCEL_SCALED;
    message_format.descriptors[1].decimation = 10;

    // Third descriptor to include
    message_format.descriptors[2].descriptor = MIP_DATA_DESC_SENSOR_GYRO_SCALED;
    message_format.descriptors[2].decimation = 10;

    microstrain_serializer serializer;

    // Create the field and init the serializer in one step
    mip_serializer_init_new_field(&serializer, _packet_view, MIP_CMD_DESC_3DM_MESSAGE_FORMAT);

    // Serialize the command
    insert_mip_3dm_message_format_command(&serializer, &message_format);

    // Update the field length
    mip_serializer_finish_new_field(&serializer, _packet_view);

    printf("Added a Message Format command field to the packet using a field and a serializer.\n");

    // Print the current state of the packet
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Poll Data command field using manual serialization
///
/// @details Shows how to create a field with a variable-length payload by
///          manually serializing parameters and handling field length updates.
///
/// @remark Field 6
///
/// @note This is similar to field 4 except with a variable-length payload.
///       This is also similar to field 5 but with lower level function calls
///       and a slightly different command.
///
/// @param _packet_view Pointer to the packet to add the field to
///
void add_poll_data_field_to_packet(mip_packet_view* _packet_view)
{
    // This part is analogous to 'mip_serializer_init_new_field' from field 3
    // It's almost the same as in field 4, except we use a zero-length field

    // Get a pointer to the payload buffer and the maximum number of bytes that can fit

    // Start by creating a zero-payload-length field and determining the remaining space
    uint8_t*  payload   = NULL;
    const int available = mip_packet_create_field(
        _packet_view,
        MIP_CMD_DESC_3DM_POLL_DATA,
        0,
        &payload
    );

    // If less than 0 bytes are available, then there wasn't even space for the field header
    assert(available > 0 && payload != NULL);

    microstrain_serializer serializer;

    // Initialize the serializer, giving it all available space
    microstrain_serializer_init_insertion(&serializer, payload, available);

    // This part is analogous to insert_mip_3dm_poll_command

    // Build the 3DM Poll Data command

    // 1. Descriptor set
    microstrain_insert_u8(&serializer, MIP_SENSOR_DATA_DESC_SET);

    // 2. Suppress_ack
    microstrain_insert_bool(&serializer, false);

    // 3. Number of data quantities
    const uint8_t num_data = 3;
    microstrain_insert_u8(&serializer, num_data);

    // 4. Array of uint8_t descriptors

    // Descriptor 1 - Reference timestamp
    microstrain_insert_u8(&serializer, MIP_DATA_DESC_SHARED_REFERENCE_TIME);

    // Descriptor 2 - Scaled Accel
    microstrain_insert_u8(&serializer, MIP_DATA_DESC_SENSOR_ACCEL_SCALED);

    // Descriptor 3 - Scaled Gyro
    microstrain_insert_u8(&serializer, MIP_DATA_DESC_SENSOR_GYRO_SCALED);

    // This part is analogous to 'mip_serializer_finish_new_field'

    // Check that everything fit in the field payload
    if (microstrain_serializer_is_ok(&serializer))
    {
        // Now we know how many bytes were written, so update the field length in the packet
        mip_packet_update_last_field_length(_packet_view, payload, (uint8_t)microstrain_serializer_length(&serializer));
    }
    // Not enough space
    else
    {
        assert(false);
        // This shouldn't happen in this example (note, there's no assertion in mip_serializer_finish_new_field)
        // Cancel the field to ensure the packet is still valid
        mip_packet_cancel_last_field(_packet_view, payload);
    }

    printf("Added a Poll Data command field to the packet using a field and a serializer.\n");

    // Print the current state of the packet
    print_packet(_packet_view);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays shared reference time field data
///
/// @details Deserializes a 64-bit nanosecond timestamp from the field payload
///          and displays it if successfully extracted. This represents the
///          device's reference time.
///
/// @param _serializer Pointer to serializer containing field data
///
void extract_shared_reference_time_field(microstrain_serializer* _serializer)
{
    uint64_t nanoseconds;
    microstrain_extract_u64(_serializer, &nanoseconds);

    if (microstrain_serializer_is_complete(_serializer))
    {
        printf("    %-20s = %" PRIu64 "\n",
            "Reference Time",
            nanoseconds
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays shared reference time delta field data
///
/// @details Deserializes a 64-bit nanosecond time difference from the field
///          payload and displays it if successfully extracted. This represents
///          the time elapsed since the last reference time.
///
/// @param _serializer Pointer to serializer containing field data
///
void extract_shared_reference_time_delta_field(microstrain_serializer* _serializer)
{
    uint64_t dt_nanoseconds;

    // Extract each value of the data field
    microstrain_extract_u64(_serializer, &dt_nanoseconds);

    // Check if the entire field was deserialized (validity check)
    if (microstrain_serializer_is_complete(_serializer))
    {
        printf("    %-20s = %" PRIu64 "\n",
            "Reference Time Delta",
            dt_nanoseconds
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays scaled accelerometer data
///
/// @details Deserializes a 3D vector of float values representing scaled
///          accelerometer measurements in m/s^2 and displays them if
///          successfully extracted.
///
/// @param _serializer Pointer to serializer containing field data
///
void extract_sensor_accel_scaled_field(microstrain_serializer* _serializer)
{
    // Note: This is not one of the recommended methods
    mip_vector3f scaled_accel_data;

    // Extract each value of the data field
    microstrain_extract_float(_serializer, &scaled_accel_data[0]);
    microstrain_extract_float(_serializer, &scaled_accel_data[1]);
    microstrain_extract_float(_serializer, &scaled_accel_data[2]);

    // Check if the entire field was deserialized (validity check)
    if (microstrain_serializer_is_complete(_serializer))
    {
        printf("    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Accel",
            scaled_accel_data[0],
            scaled_accel_data[1],
            scaled_accel_data[2]
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays scaled gyroscope data
///
/// @details Deserializes a 3D vector of float values representing scaled
///          gyroscope measurements in rad/s using the field structure. Displays
///          the values if successfully extracted.
///
/// @param _serializer Pointer to serializer containing field data
///
void extract_sensor_gyro_scaled_field(microstrain_serializer* _serializer)
{
    // Same as scaled accel except using the field data structure
    // Note: This is one of the recommended methods
    mip_sensor_scaled_gyro_data scaled_gyro_data;

    // Extract each value of the data field
    extract_mip_vector3f(_serializer, scaled_gyro_data.scaled_gyro);

    // Check if the entire field was deserialized (validity check)
    if (microstrain_serializer_is_complete(_serializer))
    {
        printf("    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            "Scaled Gyro",
            scaled_gyro_data.scaled_gyro[0],
            scaled_gyro_data.scaled_gyro[1],
            scaled_gyro_data.scaled_gyro[2]
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays delta theta (angular displacement) data
///
/// @details Deserializes a 3D vector of float values representing angular
///          displacement measurements in radians using the field structure.
///          Displays the values if successfully extracted.
///
/// @param _serializer Pointer to serializer containing field data
///
void extract_sensor_delta_theta_field(microstrain_serializer* _serializer)
{
    // Same as scaled accel except using the field data structure
    // Note: This is one of the recommended methods
    mip_sensor_delta_theta_data delta_theta_data;

    // Extract each value of the data field
    extract_mip_sensor_delta_theta_data(_serializer, &delta_theta_data);

    // Check if the entire field was deserialized (validity check)
    if (microstrain_serializer_is_complete(_serializer))
    {
        printf("    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            "Delta Theta",
            delta_theta_data.delta_theta[0],
            delta_theta_data.delta_theta[1],
            delta_theta_data.delta_theta[2]
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays delta velocity data
///
/// @details Deserializes a 3D vector of float values representing velocity
///          change measurements in m/s using the field structure. Displays
///          the values if successfully extracted.
///
/// @param _field_view Pointer to the field view containing the data
///
void extract_sensor_delta_velocity_field(const mip_field_view* _field_view)
{
    // Same as scaled accel except using the field data structure
    // Note: This is the recommended method
    mip_sensor_delta_velocity_data delta_velocity_data;

    // Extract the entire data field and check that it was deserialized (validity check)
    if (extract_mip_sensor_delta_velocity_data_from_field(_field_view, &delta_velocity_data))
    {
        printf("    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            "Delta Velocity",
            delta_velocity_data.delta_velocity[0],
            delta_velocity_data.delta_velocity[1],
            delta_velocity_data.delta_velocity[2]
        );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Creates a MIP packet from scratch with multiple fields
///
/// This function demonstrates how to create a MIP packet containing multiple
/// fields and a checksum. It creates a packet with the following structure:
/// 1. Initial checksum
/// 2. Ping command field
/// 3. Comm speed bytes field
/// 4. Comm speed field
/// 5. Comm speed serializer bytes field
/// 6. Final checksum
///
/// @details The function shows several important concepts:
///          - Creating an empty packet with a buffer
///          - Initializing a packet with a descriptor set
///          - Adding multiple fields to a packet
///          - Adding checksums at different stages
///          - Proper packet completion sequence
///
/// @note This is a demonstration function showing how to build a complex MIP
///       packet from scratch. The packet created would typically be sent to
///       a device immediately after creation.
///
/// @see initialize_empty_packet
/// @see add_checksum_to_packet
/// @see add_ping_command_to_packet
/// @see add_comm_speed_bytes_to_packet
/// @see add_comm_speed_field_to_packet
/// @see add_comm_speed_serializer_bytes_to_packet
///
void create_packet_1_from_scratch()
{
    printf("Creating packet 1 from scratch.\n\n");

    // Create a packet and an empty storage buffer for the packet
    mip_packet_view packet_view;
    uint8_t         buffer[MIP_PACKET_LENGTH_MAX] = { 0 };

    // Initialize the packet with the buffer
    initialize_empty_packet(&packet_view, buffer, sizeof(buffer) / sizeof(buffer[0]), MIP_BASE_CMD_DESC_SET);

    // Write the checksum
    add_checksum_to_packet(&packet_view);

    // Field 1
    add_ping_command_to_packet(&packet_view);

    // Field 2
    add_comm_speed_bytes_to_packet(&packet_view);

    // Field 3
    add_comm_speed_field_to_packet(&packet_view);

    // Field 4
    add_comm_speed_serializer_bytes_to_packet(&packet_view);

    // Complete packet 1
    // Write the checksum
    add_checksum_to_packet(&packet_view);

    // Note: This would be the time to send the packet to the device
    printf("Packet 1 is complete.\n\n");
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Creates two MIP packets from scratch for demonstration purposes
///
/// This function demonstrates how to create two different types of MIP packets:
/// 1. A 3DM Message Format command packet (Packet 2)
/// 2. A 3DM Poll Data command packet (Packet 3)
///
/// @details The function shows the process of:
///          - Initializing an empty packet
///          - Different ways of adding fields to the packet
///          - Adding checksums
///          - Resetting and reusing the packet buffer
///
/// @note This is a demonstration function and the packets created would
///       typically be sent to a device immediately after creation
///
/// @see initialize_empty_packet
/// @see add_message_format_field_to_packet
/// @see add_poll_data_field_to_packet
/// @see add_checksum_to_packet
/// @see mip_packet_reset
/// @see print_packet
///
void create_packet_2_and_3_from_scratch()
{
    printf("\nCreating packet 2 (3DM Message Format command) from scratch.\n\n");

    // Create a packet and an empty storage buffer for the packet
    // Note: Declared here to demonstrate resetting packets for reuse
    mip_packet_view packet_view;
    uint8_t         buffer[MIP_PACKET_LENGTH_MAX] = { 0 };

    initialize_empty_packet(&packet_view, buffer, sizeof(buffer) / sizeof(buffer[0]), MIP_BASE_CMD_DESC_SET);

    // Field 5
    add_message_format_field_to_packet(&packet_view);

    // Complete packet 2
    // Write the checksum
    add_checksum_to_packet(&packet_view);

    // Note: This would be the time to send the packet to the device
    printf("Packet 2 (3DM Message Format command) is complete.\n\n");

    const uint8_t packet_descriptor_set = MIP_3DM_CMD_DESC_SET;

    printf("\nResetting the packet for use with descriptor set 0x%02X.\n", packet_descriptor_set);

    // Start over with a new descriptor set.
    mip_packet_reset(&packet_view, packet_descriptor_set);

    // Packet is now empty and invalid again
    print_packet(&packet_view);

    printf("\nCreating packet 3 (3DM Poll Data command) from scratch.\n\n");

    // Field 6
    add_poll_data_field_to_packet(&packet_view);

    // Complete packet 3
    // Write the checksum
    add_checksum_to_packet(&packet_view);

    // Note: This would be the time to send the packet to the device
    printf("Packet 3 (3DM Poll Data command) is complete.\n\n");
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Demonstrates creating and working with a MIP packet from raw buffer
///        data
///
/// @details This function shows how to:
///          - Create a packet view from existing raw buffer data
///          - Validate the packet structure and checksum
///          - Access and display packet contents
///          - Extract specific sensor data fields based on their descriptors
///          - Process field data using the serialization tools
///
/// @remark The example uses a hardcoded packet containing multiple sensor data
///         fields for demonstration purposes.
///
/// @note This is typically not done and is used to demonstrate how to extract
///       data from a raw buffer
///
/// @see print_packet
/// @see extract_shared_reference_time_field
/// @see extract_sensor_accel_scaled_field
/// @see extract_sensor_gyro_scaled_field
/// @see extract_sensor_delta_theta_field
/// @see extract_sensor_delta_velocity_field
///
void create_packet_4_from_raw_buffer()
{
    printf("\nCreating packet 4 from a raw byte buffer.\n\n");

    mip_packet_view packet_view;
    const uint8_t raw_buffer[] = {
        0x75, 0x65, // MIP SYNC bytes

        0x80, // Packet descriptor set
        0x4C, // Packet payload length

        // Field 1
        0x0A,                                           // Field length
        0xD5,                                           // Field descriptor set
        0x00, 0x00, 0x00, 0x05, 0x5E, 0xE6, 0x7C, 0xC0, // Field raw payload

        // Field 2
        0x0A,                                           // Field length
        0xD6,                                           // Field descriptor set
        0x00, 0x00, 0x00, 0x01, 0x4E, 0x43, 0x4A, 0x00, // Field raw payload

        // Field 3
        0x0E,                                                                   // Field length
        0x04,                                                                   // Field descriptor set
        0x3D, 0x9E, 0xE8, 0x8D, 0x38, 0x7F, 0xDB, 0x00, 0xBF, 0x7A, 0xAF, 0x03, // Field raw payload

        // Field 4
        0x0E,                                                                   // Field length
        0x05,                                                                   // Field descriptor set
        0xBB, 0x0C, 0x1E, 0x30, 0xBB, 0x57, 0x2E, 0x68, 0xBB, 0xAA, 0x24, 0xAE, // Field raw payload

        // Field 5
        0x0E,                                                                   // Field length
        0x07,                                                                   // Field descriptor set
        0xBC, 0x8A, 0xAC, 0x80, 0xBC, 0x72, 0xC5, 0x0E, 0xBC, 0xC4, 0xE2, 0xC1, // Field raw payload

        // Field 6
        0x0E,                                                                   // Field length
        0x08,                                                                   // Field descriptor set
        0x3E, 0xEE, 0x3D, 0x9F, 0xBD, 0x66, 0xDA, 0xDD, 0xC0, 0xAF, 0xDE, 0xF5, // Field raw payload

        0x91, 0x96 // Packet checksum
    };

    // Create a view of the packet in the buffer.
    // Note: The buffer must not be modified, so do not call functions that manipulate the packet.
    // E.g., mip_packet_finalize(), mip_packet_add_field, mip_packet_create_field, mip_packet_reset, etc.
    mip_packet_from_buffer(&packet_view, raw_buffer, sizeof(raw_buffer) / sizeof(raw_buffer[0]));

    // Ensure the packet is valid before inspecting it.
    // This is the combination of checking:
    // 1. mip_packet_is_sane (buffer size and payload length checks)
    // 2. The packet has a non-zero descriptor set.
    // 3. The checksum is valid.
    if (!mip_packet_is_valid(&packet_view))
    {
        assert(false); // The packet should be valid in this example.
        return;
    }

    printf("Created a packet from a raw byte buffer.\n");
    print_packet(&packet_view);

    // Example of what an application might do to parse specific data
    // This example is a demonstration of the techniques used with the MIP SDK
    // consider using the "dispatch" system (see the documentation) instead

    // Only print sensor data packets
    if (mip_packet_descriptor_set(&packet_view) != MIP_SENSOR_DATA_DESC_SET)
    {
        return;
    }

    printf("Fields in the packet:\n");

    // Iterate all fields in the packet.
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    while (mip_field_next_in_packet(&field_view, &packet_view))
    {
        // Create a deserializer for the field.
        mip_serializer serializer;
        microstrain_serializer_init_extraction(&serializer, mip_field_payload(&field_view),
            mip_field_payload_length(&field_view));

        // Check what data the field contains
        switch (mip_field_field_descriptor(&field_view))
        {
            case MIP_DATA_DESC_SHARED_REFERENCE_TIME:
            {
                extract_shared_reference_time_field(&serializer);
                break;
            }
            case MIP_DATA_DESC_SHARED_REF_TIME_DELTA:
            {
                extract_shared_reference_time_delta_field(&serializer);
                break;
            }
            case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
            {
                extract_sensor_accel_scaled_field(&serializer);
                break;
            }
            case MIP_DATA_DESC_SENSOR_GYRO_SCALED:
            {
                extract_sensor_gyro_scaled_field(&serializer);
                break;
            }
            case MIP_DATA_DESC_SENSOR_DELTA_THETA:
            {
                extract_sensor_delta_theta_field(&serializer);
                break;
            }
            case MIP_DATA_DESC_SENSOR_DELTA_VELOCITY:
            {
                extract_sensor_delta_velocity_field(&field_view);
                break;
            }
            default:
            {
                break;
            }
        }
    }

    printf("\n");
}
