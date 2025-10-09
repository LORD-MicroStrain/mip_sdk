////////////////////////////////////////////////////////////////////////////////
/// @file mip_packet_example.cpp
///
/// @defgroup mip_packet_example_cpp MIP Packet Example [CPP]
///
/// @ingroup examples_cpp
///
/// @brief Example program to create raw and buffered MIP packets using C++
///
/// @details For this example, we have broken down each piece into separate
///          functions for easier documentation and is not necessary in
///          practice. This is not an exhaustive example of all MIP packet
///          features. If this example does not meet your specific setup needs,
///          please consult the MIP SDK API documentation for the proper
///          commands.
///
/// @section mip_packet_example_cpp_license License
///
/// @copyright Copyright (c) 2025 MicroStrain by HBK
///            Licensed under MIT License
///
/// @{
///

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/mip_packet.hpp>
#include <mip/mip_serialization.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/data_sensor.hpp>
#include <mip/definitions/data_shared.hpp>

#include <cassert>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>

/// @brief Whether to create packets with a user-defined buffer or not
/// @note Manual buffers are a similar approach to the C API of the MIP SDK
#define USE_MANUAL_BUFFERS false

///
/// @} group mip_packet_example_cpp
////////////////////////////////////////////////////////////////////////////////

// Print the state of a packet
static void printPacket(const mip::PacketView& _packetView);

#if USE_MANUAL_BUFFERS
// Initialize an empty packet for a given descriptor set
// Note: This is a similar approach to the C API of the MIP SDK
static mip::PacketView initializeEmptyPacket(uint8_t* _buffer, const size_t _bufferSize, const uint8_t _descriptorSet);
#else  // !USE_MANUAL_BUFFERS
// Initialize an empty packet for a given descriptor set
static mip::PacketBuf initializeEmptyPacket(const uint8_t _descriptorSet);
#endif // USE_MANUAL_BUFFERS

// Compute and add the checksum to a packet
static void addChecksumToPacket(mip::PacketView& _packetView);

// Fields added to packet 1
static void addPingCommandToPacket(mip::PacketView& _packetView);
static void addCommSpeedBytesToPacket(mip::PacketView& _packetView);
static void addCommSpeedFieldToPacket(mip::PacketView& _packetView);
static void addCommSpeedSerializerBytesToPacket(mip::PacketView& _packetView);

// Fields added to packet 2
static void addMessageFormatFieldToPacket(mip::PacketView& _packetView);

// Fields added to packet 3
static void addPollDataFieldToPacket(mip::PacketView& _packetView);

// Fields extracted from packet 4
static void extractSharedReferenceTimeField(mip::Serializer& _serializer);
static void extractSharedReferenceTimeDeltaField(mip::Serializer& _serializer);
static void extractSensorAccelScaledField(mip::Serializer& _serializer);
static void extractSensorGyroScaledField(mip::Serializer& _serializer);
static void extractSensorDeltaThetaField(mip::Serializer& _serializer);
static void extractSensorDeltaVelocityField(const mip::FieldView& _fieldView);

// Packet creation
static void createFromScratchPacket1();
static void createFromScratchPacket2And3();
static void createFromRawBufferPacket4();

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Mark printf operations as unbuffered to flush with every operation
    setvbuf(stdout, nullptr, _IONBF, 0);

    // Create packet 1 with multiple fields
    createFromScratchPacket1();

    // Create packet 2 with a single field, then reset the packet and create packet 3 with a single field
    createFromScratchPacket2And3();

    // Create packet 4 with a raw buffer and extract each field from the packet
    createFromRawBufferPacket4();

    printf("Example Completed Successfully.\n");

    printf("Press 'Enter' to exit the program.\n");

    // Make sure the console remains open
    const int confirm_exit = getc(stdin);
    (void)confirm_exit; // Unused

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup mip_packet_example_cpp
/// @{
///

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
/// @param _packetView Reference to the MIP packet view to inspect
///
static void printPacket(const mip::PacketView& _packetView)
{
    printf("Packet information:\n");

    // Ensure the packet structure is correct, otherwise a crash may result
    // This checks the following:
    //     1. The packet buffer is not NULL
    //     2. The buffer size is at least mip::PACKET_LENGTH_MIN
    //     3. The payload length does not exceed the buffer size
    // There's generally no need to check this if you're getting packets
    // directly from mip::PacketView (with sufficient buffer space) or
    // from the mip parser
    // If you're reading packets from a file, etc., without parsing, use
    // this as an inexpensive validation step
    assert(_packetView.isSane());

    // Create a buffer for printing purposes
    char packetByteBuffer[mip::PacketView::PAYLOAD_LENGTH_MAX] = {0};
    int  bufferOffset                                          = 0;

    // Get each byte in the packet, including header and checksum
    for (size_t i = 0; i < _packetView.totalLength(); i++)
    {
        bufferOffset += snprintf(
            &packetByteBuffer[bufferOffset],
            sizeof(packetByteBuffer) / sizeof(packetByteBuffer[0]) - bufferOffset,
            "%02X",
            _packetView.dataAt(i)
        );
    }

    const uint8_t payloadLength = _packetView.payloadLength();

    // Print the packet details before the fields
    printf("%4s%-20s = %u\n", " ", "Packet Length", _packetView.totalLength());
    printf("%4s%-20s = %s\n", " ", "Raw Packet", packetByteBuffer);
    printf("%4s%-20s = 0x%02X\n", " ", "MIP SYNC 1", _packetView.dataAt(mip::PacketView::Index::SYNC_1));
    printf("%4s%-20s = 0x%02X\n", " ", "MIP SYNC 2", _packetView.dataAt(mip::PacketView::Index::SYNC_2));
    printf("%4s%-20s = 0x%02X\n", " ", "Descriptor Set", _packetView.descriptorSet());
    printf("%4s%-20s = 0x%02X\n", " ", "Payload Length", payloadLength);

    // Check that fields exist using the payload length
    if (payloadLength > 0)
    {
        printf("%4sFields:", " ");
    }

    // Iterate the packet and extract each field
    for (const mip::FieldView& fieldView : _packetView)
    {
        printf("\n");

        // Print descriptors (the descriptor set always matches the packet)
        // Include the size of the length byte
        printf("%8s%-16s = 0x%02X\n", " ", "Field Length", fieldView.totalLength());
        printf("%8s%-16s = 0x%02X\n", " ", "Field Descriptor", fieldView.fieldDescriptor());
        printf("%8s%-16s = ", " ", "Raw Payload");

        // Print field payload bytes.
        for (size_t i = 0; i < fieldView.payloadLength(); i++)
        {
            printf("%02X", fieldView.payload(i));
        }

        printf("\n");
    }

    // Print the checksum most and least significant bytes, and if it's valid or not
    const uint16_t checksumValue = _packetView.checksumValue();
    printf("%4sChecksum (%s):\n", " ", _packetView.isValid() ? "Valid" : "Invalid");
    printf("%8s%-16s = 0x%02X\n", " ", "MSB", checksumValue >> 0x08);
    printf("%8s%-16s = 0x%02X\n\n", " ", "LSB", checksumValue & 0xFF);
}

#if USE_MANUAL_BUFFERS
////////////////////////////////////////////////////////////////////////////////
/// @brief Creates an empty MIP packet for manual construction
///
/// @details Initializes a MIP packet in a user-provided buffer with the
///          specified descriptor set. Demonstrates low-level packet creation
///          using the PacketView class for manual buffer management. The
///          packet is initially empty and invalid until fields are added.
///
/// @note This is a similar approach to the C API of the MIP SDK
///
/// @param _buffer Buffer to store the packet data
/// @param _bufferSize Size of the buffer in bytes
/// @param _descriptorSet Descriptor set to use for the packet
///
/// @return A PacketView referencing the initialized packet
///
static mip::PacketView initializeEmptyPacket(uint8_t* _buffer, const size_t _bufferSize, const uint8_t _descriptorSet)
{
    // If the descriptor set is specified, even if it's invalid, i.e., 0x00,
    // the constructor initializes a new MIP packet in the buffer.
    // Otherwise, it assumes the buffer already contains a valid MIP packet.
    const mip::PacketView packetView(_buffer, _bufferSize, _descriptorSet);

    printf("Created an empty packet for descriptor set 0x%02X.\n", _descriptorSet);

    // Print the current state of the packet
    // Note: The packet is currently empty and invalid
    printPacket(packetView);

    return packetView;
}
#else  // !USE_MANUAL_BUFFERS
////////////////////////////////////////////////////////////////////////////////
/// @brief Creates an empty MIP packet using automatic buffer management
///
/// @details Initializes a MIP packet using the PacketBuf class which handles
///          buffer allocation internally. This demonstrates the higher-level
///          packet creation API. The packet is initially empty and invalid
///          until fields are added.
///
/// @param _descriptorSet Descriptor set to use for the packet
///
/// @return A PacketBuf containing the initialized packet
///
static mip::PacketBuf initializeEmptyPacket(const uint8_t _descriptorSet)
{
    // If the descriptor set is specified, even if it's invalid, i.e., 0x00,
    // the constructor initializes a new MIP packet in the buffer.
    // Otherwise, it assumes the buffer already contains a valid MIP packet.
    const mip::PacketBuf packetBuf(_descriptorSet);

    printf("Created an empty packet for descriptor set 0x%02X.\n", _descriptorSet);

    // Print the current state of the packet
    // Note: The packet is currently empty and invalid
    printPacket(packetBuf);

    return packetBuf;
}
#endif // USE_MANUAL_BUFFERS

////////////////////////////////////////////////////////////////////////////////
/// @brief Computes and adds a checksum to a MIP packet
///
/// @details Finalizes the packet by computing and appending the checksum.
///          This should be called after all fields have been added to the
///          packet.
///
/// @param _packetView Reference to the packet to finalize with checksum
///
static void addChecksumToPacket(mip::PacketView& _packetView)
{
    _packetView.finalize();

    printf("Added a checksum to the packet.\n");

    // Print the current state of the packet
    // Note: The packet is now validated
    printPacket(_packetView);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a Ping command field to a MIP packet
///
/// @details Creates a field with the Base Ping command descriptor (0x01) with
///          no payload data.
///
/// @remark Field 1
///
/// @param _packetView Reference to the packet to add the field to
///
static void addPingCommandToPacket(mip::PacketView& _packetView)
{
    _packetView.addField(
        mip::commands_base::Ping::FIELD_DESCRIPTOR, // Field descriptor set
        nullptr,                                    // Payload data
        0                                           // Payload buffer length
    );

    printf("Added a %s command field to the packet.\n", mip::commands_base::Ping::DOC_NAME);

    // Print the current state of the packet
    printPacket(_packetView);
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
/// @param _packetView Reference to the packet to add the field to
///
static void addCommSpeedBytesToPacket(mip::PacketView& _packetView)
{
    // Build the raw payload for the packet
    const uint8_t commSpeedPayload[] = {
        // Function selector
        0x01,
        // Port
        0x01,
        // Baudrate
        0x00,
        0x01,
        0xC2,
        0x00
    };

    _packetView.addField(
        mip::commands_base::CommSpeed::FIELD_DESCRIPTOR,       // Field descriptor set
        commSpeedPayload,                                      // Payload data
        sizeof(commSpeedPayload) / sizeof(commSpeedPayload[0]) // Payload buffer length
    );

    printf("Added a %s command field to the packet using raw bytes.\n", mip::commands_base::CommSpeed::DOC_NAME);

    // Print the current state of the packet
    printPacket(_packetView);
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
/// @param _packetView Reference to the packet to add the field to
///
static void addCommSpeedFieldToPacket(mip::PacketView& _packetView)
{
    mip::commands_base::CommSpeed commSpeed;
    commSpeed.function = mip::FunctionSelector::WRITE;
    commSpeed.port     = 0x01;
    commSpeed.baud     = 115200;

    _packetView.addField(commSpeed);

    printf("Added a %s command field to the packet using a field.\n", mip::commands_base::CommSpeed::DOC_NAME);

    // Print the current state of the packet
    printPacket(_packetView);
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
/// @note This is exactly the same as field 3 but without the helper functions.
///       This is intended to show what happens "behind the scenes".
///
/// @param _packetView Reference to the packet to add the field to
///
static void addCommSpeedSerializerBytesToPacket(mip::PacketView& _packetView)
{
    // Create a field and get the payload pointer
    // The return value is the number of bytes remaining after allocating this field
    // Note: We know the field length will be 6 bytes, allowing us to not have to update
    // the field length after initialization as field 6 does
    mip::Serializer serializer = _packetView.createField(
        mip::commands_base::CommSpeed::FIELD_DESCRIPTOR,
        6 // Payload length
    );

    printf("Reserved space in the packet for a %s command field.\n", mip::commands_base::CommSpeed::DOC_NAME);

    // Show what the packet looks like after allocating a field
    // The header will exist, but the payload will not be valid yet
    printPacket(_packetView);

    // Write parameters to the payload
    // Note: This is analogous to mip::commands_base::CommSpeed::insert
    serializer.insert<uint8_t>(static_cast<uint8_t>(mip::FunctionSelector::WRITE));
    serializer.insert<uint8_t>(0x01);
    serializer.insert<uint32_t>(115200);

    // Make sure the serializer is not out of space
    assert(serializer.isOk());

    printf("Added a %s command field to the packet using a serializer.\n", mip::commands_base::CommSpeed::DOC_NAME);

    // Print the current state of the packet
    printPacket(_packetView);
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
/// @param _packetView Reference to the packet to add the field to
///
static void addMessageFormatFieldToPacket(mip::PacketView& _packetView)
{
    mip::commands_3dm::MessageFormat messageFormat;

    // Mark the command for writing
    messageFormat.function = mip::FunctionSelector::WRITE;

    // Set the message format data descriptor set
    messageFormat.desc_set = mip::data_sensor::DESCRIPTOR_SET;

    // Number of descriptors to include
    messageFormat.num_descriptors = 3;

    // First descriptor to include
    messageFormat.descriptors[0] = {
        mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR,
        10 // Decimation
    };

    // Second descriptor to include
    messageFormat.descriptors[1] = {
        mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR,
        10 // Decimation
    };

    // Third descriptor to include
    messageFormat.descriptors[2] = {
        mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR,
        10 // Decimation
    };

    _packetView.addField(messageFormat);

    printf("Added a %s command field to the packet using a field.\n", mip::commands_3dm::MessageFormat::DOC_NAME);

    // Print the current state of the packet
    printPacket(_packetView);
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
/// @param _packetView Reference to the packet to add the field to
///
static void addPollDataFieldToPacket(mip::PacketView& _packetView)
{
    // Create a field of unknown length
    mip::PacketView::AllocatedField pollData = _packetView.createField(mip::commands_3dm::PollData::FIELD_DESCRIPTOR);

    // Create the field data
    constexpr uint8_t descriptors[3] = {
        mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR, // Data field descriptor set
        mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR,        // Data field descriptor set
        mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR          // Data field descriptor set
    };

    pollData.insert(
        mip::commands_3dm::PollData::DESCRIPTOR_SET, // Data descriptor set
        false,                                       // Suppress ACK
        sizeof(descriptors) / sizeof(descriptors[0]) // Number of descriptors
    );

    pollData.insert(descriptors, sizeof(descriptors) / sizeof(descriptors[0]));

    // Update the field length
    // Note: If the field exceeds the remaining space in the packet, this will
    // instead remove the field entirely and return false
    const bool ok = pollData.commit();
    (void)ok;

    // An assertion shouldn't happen in this example but is a good check to have
    assert(ok);

    printf("Added a %s command field to the packet using AllocatedField.\n", mip::commands_3dm::PollData::DOC_NAME);

    // Print the current state of the packet
    printPacket(_packetView);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays shared reference time field data
///
/// @details Deserializes a 64-bit nanosecond timestamp from the field payload
///          and displays it if successfully extracted. This represents the
///          device's reference time.
///
/// @param _serializer Reference to the serializer containing the field data
///
static void extractSharedReferenceTimeField(mip::Serializer& _serializer)
{
    uint64_t nanoseconds;

    // Extract each value of the data field
    if (_serializer.extract(nanoseconds))
    {
        printf("    %-20s = %" PRIu64 "\n", mip::data_shared::ReferenceTimestamp::DOC_NAME, nanoseconds);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays shared reference time delta field data
///
/// @details Deserializes a 64-bit nanosecond time difference from the field
///          payload and displays it if successfully extracted. This represents
///          the time elapsed since the last reference time.
///
/// @param _serializer Reference to the serializer containing the field data
///
static void extractSharedReferenceTimeDeltaField(mip::Serializer& _serializer)
{
    uint64_t dtNanoseconds;

    // Extract each value of the data field
    if (_serializer.extract(dtNanoseconds))
    {
        printf("    %-20s = %" PRIu64 "\n", mip::data_shared::ReferenceTimeDelta::DOC_NAME, dtNanoseconds);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Extracts and displays scaled accelerometer data
///
/// @details Deserializes a 3D vector of float values representing scaled
///          accelerometer measurements in m/s^2 and displays them if
///          successfully extracted.
///
/// @param _serializer Reference to the serializer containing the field data
///
static void extractSensorAccelScaledField(mip::Serializer& _serializer)
{
    // Note: This is not one of the recommended methods
    mip::Vector3f scaledAccelData;

    // Extract each value of the data field
    if (_serializer.extract(scaledAccelData))
    {
        printf(
            "    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            mip::data_sensor::ScaledAccel::DOC_NAME,
            scaledAccelData[0],
            scaledAccelData[1],
            scaledAccelData[2]
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
/// @param _serializer Reference to the serializer containing the field data
///
static void extractSensorGyroScaledField(mip::Serializer& _serializer)
{
    // Same as scaled accel except using the field data structure
    // Note: This is one of the recommended methods
    mip::data_sensor::ScaledGyro scaledGyroData;

    // Extract each value of the data field
    if (_serializer.extract(scaledGyroData))
    {
        printf(
            "    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            mip::data_sensor::ScaledGyro::DOC_NAME,
            scaledGyroData.scaled_gyro[0],
            scaledGyroData.scaled_gyro[1],
            scaledGyroData.scaled_gyro[2]
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
/// @param _serializer Reference to the serializer containing the field data
///
static void extractSensorDeltaThetaField(mip::Serializer& _serializer)
{
    // Same as scaled accel except using the field data structure
    // Note: This is one of the recommended methods
    mip::data_sensor::DeltaTheta deltaThetaData;

    // Extract each value of the data field
    if (_serializer.extract(deltaThetaData))
    {
        printf(
            "    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            mip::data_sensor::DeltaTheta::DOC_NAME,
            deltaThetaData.delta_theta[0],
            deltaThetaData.delta_theta[1],
            deltaThetaData.delta_theta[2]
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
/// @param _fieldView Reference to the field view containing the data
///
static void extractSensorDeltaVelocityField(const mip::FieldView& _fieldView)
{
    // Same as scaled accel except using the field data structure
    // Note: This is the recommended method
    mip::data_sensor::DeltaVelocity deltaVelocityData;

    // Extract the entire data field and check that it was deserialized (validity check)
    if (_fieldView.extract(deltaVelocityData))
    {
        printf(
            "    %-20s = [%9.6f, %9.6f, %9.6f]\n",
            mip::data_sensor::DeltaVelocity::DOC_NAME,
            deltaVelocityData.delta_velocity[0],
            deltaVelocityData.delta_velocity[1],
            deltaVelocityData.delta_velocity[2]
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
/// @see initializeEmptyPacket
/// @see addChecksumToPacket
/// @see addPingCommandToPacket
/// @see addCommSpeedBytesToPacket
/// @see addCommSpeedFieldToPacket
/// @see addCommSpeedSerializerBytesToPacket
///
static void createFromScratchPacket1()
{
    printf("Creating packet 1 from scratch.\n\n");

    constexpr uint8_t packetDescriptorSet = mip::commands_base::DESCRIPTOR_SET;

#if USE_MANUAL_BUFFERS
    // Create a packet and an empty storage buffer for the packet
    // Note: This approach is similar to the C API
    uint8_t         buffer[mip::PacketView::PACKET_SIZE_MAX] = {0};
    mip::PacketView packet = initializeEmptyPacket(buffer, sizeof(buffer) / sizeof(buffer[0]), packetDescriptorSet);
#else  // !USE_MANUAL_BUFFERS
    // Create a packet using a packet buffer object
    mip::PacketBuf packet = initializeEmptyPacket(packetDescriptorSet);
#endif // USE_MANUAL_BUFFERS

    // Write the checksum
    addChecksumToPacket(packet);

    // Field 1
    addPingCommandToPacket(packet);

    // Field 2
    addCommSpeedBytesToPacket(packet);

    // Field 3
    addCommSpeedFieldToPacket(packet);

    // Field 4
    addCommSpeedSerializerBytesToPacket(packet);

    // Complete packet 1
    // Write the checksum
    addChecksumToPacket(packet);

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
/// @see initializeEmptyPacket
/// @see addMessageFormatFieldToPacket
/// @see addPollDataFieldToPacket
/// @see addChecksumToPacket
/// @see mip::PacketView::reset
/// @see printPacket
///
static void createFromScratchPacket2And3()
{
    printf("\nCreating packet 2 (3DM Message Format command) from scratch.\n\n");

    uint8_t packetDescriptorSet = mip::commands_3dm::DESCRIPTOR_SET;

#if USE_MANUAL_BUFFERS
    // Create a packet and an empty storage buffer for the packet
    // Note: Declared here to demonstrate resetting packets for reuse
    // Note: This approach is similar to the C API
    uint8_t         buffer[mip::PacketView::PACKET_SIZE_MAX] = {0};
    mip::PacketView packet = initializeEmptyPacket(buffer, sizeof(buffer) / sizeof(buffer[0]), packetDescriptorSet);
#else  // !USE_MANUAL_BUFFERS
    // Create a packet using a packet buffer object
    // Note: Declared here to demonstrate resetting packets for reuse
    mip::PacketBuf packet = initializeEmptyPacket(packetDescriptorSet);
#endif // USE_MANUAL_BUFFERS

    // Field 5
    addMessageFormatFieldToPacket(packet);

    // Complete packet 2
    // Write the checksum
    addChecksumToPacket(packet);

    // Note: This would be the time to send the packet to the device
    printf("Packet 2 (%s command) is complete.\n\n", mip::commands_3dm::MessageFormat::DOC_NAME);

    packetDescriptorSet = mip::commands_3dm::DESCRIPTOR_SET;

    // Start over with a new descriptor set.
    packet.reset(packetDescriptorSet);

    printf("\nReset the packet for use with descriptor set 0x%02X.\n", packetDescriptorSet);

    // Packet is now empty and invalid again
    printPacket(packet);

    printf("\nCreating packet 3 (%s command) from scratch.\n\n", mip::commands_3dm::PollData::DOC_NAME);

    // Field 6
    addPollDataFieldToPacket(packet);

    // Complete packet 3
    // Write the checksum
    addChecksumToPacket(packet);

    // Note: This would be the time to send the packet to the device
    printf("Packet 3 (%s command) is complete.\n\n", mip::commands_3dm::PollData::DOC_NAME);
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
/// @see printPacket
/// @see extractSharedReferenceTimeField
/// @see extractSensorAccelScaledField
/// @see extractSensorGyroScaledField
/// @see extractSensorDeltaThetaField
/// @see extractSensorDeltaVelocityField
///
static void createFromRawBufferPacket4()
{
    printf("\nCreating packet 4 from a raw byte buffer.\n\n");

    const uint8_t rawBuffer[] = {
        0x75, 0x65, // MIP SYNC bytes

        0x80, // Packet descriptor set
        0x4C, // Packet payload length

        0x0A,                                           // Field 1 length
        0xD5,                                           // Field 1 descriptor set
        0x00, 0x00, 0x00, 0x05, 0x5E, 0xE6, 0x7C, 0xC0, // Field 1 raw payload

        0x0A,                                           // Field 2 length
        0xD6,                                           // Field 2 descriptor set
        0x00, 0x00, 0x00, 0x01, 0x4E, 0x43, 0x4A, 0x00, // Field 2 raw payload

        0x0E,                                                                   // Field 3 length
        0x04,                                                                   // Field 3 descriptor set
        0x3D, 0x9E, 0xE8, 0x8D, 0x38, 0x7F, 0xDB, 0x00, 0xBF, 0x7A, 0xAF, 0x03, // Field 3 raw payload

        0x0E,                                                                   // Field 4 length
        0x05,                                                                   // Field 4 descriptor set
        0xBB, 0x0C, 0x1E, 0x30, 0xBB, 0x57, 0x2E, 0x68, 0xBB, 0xAA, 0x24, 0xAE, // Field 4 raw payload

        0x0E,                                                                   // Field 5 length
        0x07,                                                                   // Field 5 descriptor set
        0xBC, 0x8A, 0xAC, 0x80, 0xBC, 0x72, 0xC5, 0x0E, 0xBC, 0xC4, 0xE2, 0xC1, // Field 5 raw payload

        0x0E,                                                                   // Field 6 length
        0x08,                                                                   // Field 6 descriptor set
        0x3E, 0xEE, 0x3D, 0x9F, 0xBD, 0x66, 0xDA, 0xDD, 0xC0, 0xAF, 0xDE, 0xF5, // Field 6 raw payload

        0x91, 0x96 // Packet checksum
    };

    // Create a view of the packet in the buffer.
    // Note: The buffer must not be modified, so do not call functions that manipulate the packet.
    // E.g. finalize(), addField, createField, reset, etc.
    const mip::PacketView packetView(rawBuffer, sizeof(rawBuffer) / sizeof(rawBuffer[0]));

    // Ensure the packet is valid before inspecting it.
    // This is the combination of checking:
    // 1. packet.isSane() (buffer size and payload length checks)
    // 2. The packet has a non-zero descriptor set.
    // 3. The checksum is valid.
    if (!packetView.isValid())
    {
        assert(false); // The packet should be valid in this example.
        return;
    }

    printf("Created a packet from a raw byte buffer.\n");
    printPacket(packetView);

    // Example of what an application might do to parse specific data
    // This example is a demonstration of the techniques used with the MIP SDK
    // consider using the "dispatch" system (see the documentation) instead

    // Only print sensor data packets
    if (packetView.descriptorSet() != mip::data_sensor::DESCRIPTOR_SET)
    {
        return;
    }

    printf("Fields in the packet:\n");

    // Iterate all fields in the packet.
    for (const mip::FieldView& fieldView : packetView)
    {
        // Create a deserializer for the field.
        mip::Serializer serializer(fieldView.payload(), fieldView.payloadLength());

        // Check what data the field contains
        switch (fieldView.fieldDescriptor())
        {
            case mip::data_shared::ReferenceTimestamp::FIELD_DESCRIPTOR:
            {
                extractSharedReferenceTimeField(serializer);
                break;
            }
            case mip::data_shared::ReferenceTimeDelta::FIELD_DESCRIPTOR:
            {
                extractSharedReferenceTimeDeltaField(serializer);
                break;
            }
            case mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR:
            {
                extractSensorAccelScaledField(serializer);
                break;
            }
            case mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR:
            {
                extractSensorGyroScaledField(serializer);
                break;
            }
            case mip::data_sensor::DeltaTheta::FIELD_DESCRIPTOR:
            {
                extractSensorDeltaThetaField(serializer);
                break;
            }
            case mip::data_sensor::DeltaVelocity::FIELD_DESCRIPTOR:
            {
                extractSensorDeltaVelocityField(fieldView);
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

///
/// @} group mip_packet_example_cpp
////////////////////////////////////////////////////////////////////////////////
