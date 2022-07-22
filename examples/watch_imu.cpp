
#include "example_utils.hpp"

#include <mscl/mip/mip_result.h>
#include <mscl/mip/mip_dispatch.h>

#include <mscl/mip/definitions/commands_base.h>
#include <mscl/mip/definitions/commands_3dm.h>
#include <mscl/mip/definitions/data_sensor.h>
#include <mscl/mip/mip.hpp>

#include <thread>


void handlePacket(void*, const mscl::C::mip_packet* packet_, mscl::Timestamp timestamp)
{
    mscl::MipPacket packet(*packet_);

    // if(packet.descriptorSet() != mscl::MIP_SENSOR_DATA_DESC_SET)
    //     return;

    printf("\nGot packet with descriptor set 0x%02X:", packet.descriptorSet());

    for(mscl::MipField field : packet)
        printf(" %02X", field.fieldDescriptor());

    printf("\n");
}

void handleAccel(void*, const mscl::C::mip_field* field_, mscl::Timestamp timestamp)
{
    mscl::MipField field(*field_);
    mscl::C::mip_sensor_scaled_accel_data data;

    size_t readBytes = mscl::C::extract_mip_sensor_scaled_accel_data(field.payload(), field.payloadLength(), 0, &data);

    if(readBytes == field.payloadLength())
        printf("Accel Data: %f, %f, %f\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2]);
}

void handleGyro(void*, const mscl::C::mip_field* field_, mscl::Timestamp timestamp)
{
    mscl::MipField field(*field_);
    mscl::C::mip_sensor_scaled_gyro_data data;

    size_t readBytes = mscl::C::extract_mip_sensor_scaled_gyro_data(field.payload(), field.payloadLength(), 0, &data);

    if(readBytes == field.payloadLength())
        printf("Gyro Data:  %f, %f, %f\n", data.scaled_gyro[0], data.scaled_gyro[1], data.scaled_gyro[2]);
}

void handleMag(void*, const mscl::C::mip_field* field_, mscl::Timestamp timestamp)
{
    mscl::MipField field(*field_);
    mscl::C::mip_sensor_scaled_mag_data data;

    size_t readBytes = mscl::C::extract_mip_sensor_scaled_mag_data(field.payload(), field.payloadLength(), 0, &data);

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
        result = mscl::C::get_data_base_rate(device.get(), mscl::MIP_SENSOR_DATA_DESC_SET, &base_rate);

        if( result != mscl::C::MIP_ACK_OK )
            return fprintf(stderr, "Failed to get base rate: %s (%d)\n", result.name(), result.value), 1;

        // Set the message format to stream at 100 Hz.

        const uint16_t sample_rate = 100; // Hz
        const uint16_t decimation = base_rate / sample_rate;

        std::array<mscl::MipDescriptorRate, 3> descriptors = {{
            { mscl::MIP_DATA_DESC_SENSOR_ACCEL_SCALED, decimation },
            { mscl::MIP_DATA_DESC_SENSOR_GYRO_SCALED,  decimation },
            { mscl::MIP_DATA_DESC_SENSOR_MAG_SCALED,   decimation },
        }};

        result = mscl::C::write_mip_3dm_message_format(device.get(), mscl::MIP_SENSOR_DATA_DESC_SET, descriptors.size(), descriptors.data());

        if( result == mscl::C::MIP_NACK_INVALID_PARAM )
        {
            // Failed to set message format - maybe this device doesn't have a magnetometer.
            // Try again without the last descriptor (scaled mag).
            result = mscl::C::write_mip_3dm_message_format(device.get(), mscl::MIP_SENSOR_DATA_DESC_SET, descriptors.size()-1, descriptors.data());
        }
        if( result != mscl::C::MIP_ACK_OK )
            return fprintf(stderr, "Failed to set message format: %s (%d)\n", result.name(), result.value), 1;

        // Register some callbacks.
        mscl::C::mip_dispatch_handler packetHandler;
        mscl::C::mip_dispatch_handler dataHandlers[3];
        mscl::C::mip_interface_register_packet_callback(device.get(), &packetHandler, mscl::C::MIP_DISPATCH_DESCSET_DATA, &handlePacket, NULL);
        mscl::C::mip_interface_register_field_callback(device.get(), &dataHandlers[0], mscl::MIP_SENSOR_DATA_DESC_SET, mscl::MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &handleAccel, NULL);
        mscl::C::mip_interface_register_field_callback(device.get(), &dataHandlers[1], mscl::MIP_SENSOR_DATA_DESC_SET, mscl::MIP_DATA_DESC_SENSOR_GYRO_SCALED , &handleGyro , NULL);
        mscl::C::mip_interface_register_field_callback(device.get(), &dataHandlers[2], mscl::MIP_SENSOR_DATA_DESC_SET, mscl::MIP_DATA_DESC_SENSOR_MAG_SCALED  , &handleMag  , NULL);

        // Resume the device to ensure it's streaming.

        result = mscl::C::resume(device.get());
        if( result != mscl::C::MIP_ACK_OK )
            return fprintf(stderr, "Failed to resume device: %s (%d)\n", result.name(), result.value), 1;

        // Process data for 3 seconds.
        for(unsigned int i=0; i<30; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            device->update();
        }

        result = mscl::C::set_to_idle(device.get());
        if( result != mscl::C::MIP_ACK_OK )
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
