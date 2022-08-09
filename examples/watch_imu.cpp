
#include "example_utils.hpp"

#include <mscl/mip/mip_result.h>
#include <mscl/mip/mip_dispatch.h>
#include <mscl/utils/serialization.h>

#include <mscl/mip/definitions/commands_base.hpp>
#include <mscl/mip/definitions/commands_3dm.hpp>
#include <mscl/mip/definitions/data_sensor.hpp>
#include <mscl/mip/mip.hpp>

#include <thread>


void handlePacket(void*, const mscl::MipPacket& packet, mscl::Timestamp timestamp)
{
    // if(packet.descriptorSet() != mscl::MIP_SENSOR_DATA_DESC_SET)
    //     return;

    printf("\nGot packet with descriptor set 0x%02X:", packet.descriptorSet());

    for(mscl::MipField field : packet)
        printf(" %02X", field.fieldDescriptor());

    printf("\n");
}

void handleAccel(void*, const mscl::MipField& field, mscl::Timestamp timestamp)
{
    mscl::data_sensor::ScaledAccel data;

    if( mscl::extract(data, field.payload(), field.payloadLength()) )
        printf("Accel Data: %f, %f, %f\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2]);
}

void handleGyro(void*, const mscl::MipField& field, mscl::Timestamp timestamp)
{
    mscl::data_sensor::ScaledGyro data;

    if( mscl::extract(data, field.payload(), field.payloadLength()) )
        printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void*, const mscl::MipField& field, mscl::Timestamp timestamp)
{
    mscl::data_sensor::ScaledMag data;

    if( mscl::extract(data, field.payload(), field.payloadLength()) )
        printf("Mag Data:   %f, %f, %f\n", data.scaled_mag[0], data.scaled_mag[1], data.scaled_mag[2]);
}


int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<mscl::MipDeviceInterface> device = handleCommonArgs(argc, argv);

        mscl::MipCmdResult result;

        // Get the base rate.

        uint16_t base_rate;
        result = mscl::commands_3dm::getBaseRate(*device, mscl::data_sensor::DESCRIPTOR_SET, base_rate);

        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to get base rate: %s (%d)\n", result.name(), result.value), 1;

        // Set the message format to stream at 100 Hz.

        const uint16_t sample_rate = 100; // Hz
        const uint16_t decimation = base_rate / sample_rate;

        std::array<mscl::MipDescriptorRate, 3> descriptors = {{
            { mscl::data_sensor::DATA_ACCEL_SCALED, decimation },
            { mscl::data_sensor::DATA_GYRO_SCALED,  decimation },
            { mscl::data_sensor::DATA_MAG_SCALED,   decimation },
        }};

        result = mscl::commands_3dm::writeMessageFormat(*device, mscl::data_sensor::DESCRIPTOR_SET, descriptors.size(), descriptors.data());

        if( result == mscl::MipCmdResult::NACK_COMMAND_FAILED )
        {
            // Failed to set message format - maybe this device doesn't have a magnetometer.
            // Try again without the last descriptor (scaled mag).
            result = mscl::commands_3dm::writeMessageFormat(*device, mscl::data_sensor::DESCRIPTOR_SET, descriptors.size()-1, descriptors.data());
        }
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to set message format: %s (%d)\n", result.name(), result.value), 1;

        // Register some callbacks.

        mscl::MipDispatchHandler packetHandler;
        device->registerPacketCallback<&handlePacket>(packetHandler, mscl::C::MIP_DISPATCH_DESCSET_DATA);

        mscl::MipDispatchHandler dataHandlers[3];
        device->registerFieldCallback<&handleAccel>(dataHandlers[0], mscl::data_sensor::DESCRIPTOR_SET, mscl::data_sensor::DATA_ACCEL_SCALED);
        device->registerFieldCallback<&handleGyro >(dataHandlers[1], mscl::data_sensor::DESCRIPTOR_SET, mscl::data_sensor::DATA_GYRO_SCALED );
        device->registerFieldCallback<&handleMag  >(dataHandlers[2], mscl::data_sensor::DESCRIPTOR_SET, mscl::data_sensor::DATA_MAG_SCALED  );

        // Enable the data stream and resume the device.

        result = mscl::commands_3dm::writeDatastreamControl(*device, mscl::data_sensor::DESCRIPTOR_SET, true);
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to enable datastream: %s (%d)\n", result.name(), result.value), 1;

        // Resume the device to ensure it's streaming.

        result = mscl::commands_base::resume(*device);
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to resume device: %s (%d)\n", result.name(), result.value), 1;

        // Process data for 3 seconds.
        const mscl::Timestamp start_time = getCurrentTimestamp();
        do
        {
            device->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        } while( getCurrentTimestamp() - start_time < 3000 );

        result = mscl::commands_base::setIdle(*device);
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to idle device: %s (%d)\n", result.name(), result.value), 1;

        // mscl::TdmCommands::EventControl ctrl;
        // ctrl.mode = mscl::TdmCommands::EventControl::Mode::ENABLED;

        return 0;
    }
    catch(const std::underflow_error& ex)
    {
        return printCommonUsage(argv);
    }
    catch(const std::exception& ex)
    {
        fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }
    return 0;
}
