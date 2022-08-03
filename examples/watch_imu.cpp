
#include "example_utils.hpp"

#include <mscl/mip/mip_result.h>
#include <mscl/mip/mip_dispatch.h>

#include <mscl/mip/definitions/commands_base.h>
#include <mscl/mip/definitions/commands_3dm.h>
#include <mscl/mip/definitions/data_sensor.h>
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
    mscl::SensorData::ScaledAccel data;

    size_t readBytes = data.extract(field.payload(), field.payloadLength(), 0);

    if(readBytes == field.payloadLength())
        printf("Accel Data: %f, %f, %f\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2]);
}

void handleGyro(void*, const mscl::MipField& field, mscl::Timestamp timestamp)
{
    mscl::SensorData::ScaledGyro data;

    size_t readBytes = data.extract(field.payload(), field.payloadLength(), 0);

    if(readBytes == field.payloadLength())
        printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void*, const mscl::MipField& field, mscl::Timestamp timestamp)
{
    mscl::SensorData::ScaledMag data;

    size_t readBytes = data.extract(field.payload(), field.payloadLength(), 0);

    if(readBytes == field.payloadLength())
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
        result = mscl::TdmCommands::getBaseRate(*device, mscl::MIP_SENSOR_DATA_DESC_SET, base_rate);

        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to get base rate: %s (%d)\n", result.name(), result.value), 1;

        // Set the message format to stream at 100 Hz.

        const uint16_t sample_rate = 100; // Hz
        const uint16_t decimation = base_rate / sample_rate;

        std::array<mscl::MipDescriptorRate, 3> descriptors = {{
            { mscl::MIP_DATA_DESC_SENSOR_ACCEL_SCALED, decimation },
            { mscl::MIP_DATA_DESC_SENSOR_GYRO_SCALED,  decimation },
            { mscl::MIP_DATA_DESC_SENSOR_MAG_SCALED,   decimation },
        }};

        result = mscl::TdmCommands::writeMessageFormat(*device, mscl::MIP_SENSOR_DATA_DESC_SET, descriptors.size(), descriptors.data());

        if( result == mscl::MipCmdResult::NACK_COMMAND_FAILED )
        {
            // Failed to set message format - maybe this device doesn't have a magnetometer.
            // Try again without the last descriptor (scaled mag).
            result = mscl::TdmCommands::writeMessageFormat(*device, mscl::MIP_SENSOR_DATA_DESC_SET, descriptors.size()-1, descriptors.data());
        }
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to set message format: %s (%d)\n", result.name(), result.value), 1;

        // Register some callbacks.

        mscl::MipDispatchHandler packetHandler;
        device->registerPacketCallback<&handlePacket>(packetHandler, mscl::C::MIP_DISPATCH_DESCSET_DATA);

        mscl::MipDispatchHandler dataHandlers[3];
        device->registerFieldCallback<&handleAccel>(dataHandlers[0], mscl::MIP_SENSOR_DATA_DESC_SET, mscl::MIP_DATA_DESC_SENSOR_ACCEL_SCALED);
        device->registerFieldCallback<&handleGyro >(dataHandlers[1], mscl::MIP_SENSOR_DATA_DESC_SET, mscl::MIP_DATA_DESC_SENSOR_GYRO_SCALED );
        device->registerFieldCallback<&handleMag  >(dataHandlers[2], mscl::MIP_SENSOR_DATA_DESC_SET, mscl::MIP_DATA_DESC_SENSOR_MAG_SCALED  );

        // Enable the data stream and resume the device.

        result = mscl::TdmCommands::writeDatastreamControl(*device, mscl::MIP_SENSOR_DATA_DESC_SET, true);
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to enable datastream: %s (%d)\n", result.name(), result.value), 1;

        // Resume the device to ensure it's streaming.

        result = mscl::BaseCommands::resume(*device);
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to resume device: %s (%d)\n", result.name(), result.value), 1;

        // Process data for 3 seconds.
        const mscl::Timestamp start_time = getCurrentTimestamp();
        do
        {
            device->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        } while( getCurrentTimestamp() - start_time < 3000 );

        result = mscl::BaseCommands::setIdle(*device);
        if( result != mscl::MipCmdResult::ACK_OK )
            return fprintf(stderr, "Failed to idle device: %s (%d)\n", result.name(), result.value), 1;

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
