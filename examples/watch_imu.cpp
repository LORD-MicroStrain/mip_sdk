
#include "example_utils.hpp"

#include <mip/mip_result.h>

#include <mip/definitions/commands_base.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/data_sensor.h>


int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<mscl::MipDeviceInterface> device = handleCommonArgs(argc, argv);

        mscl::MipCmdResult result;

        // Get the base rate.

        uint16_t base_rate;
        result = mscl::get_data_base_rate(device.get(), mscl::MIP_SENSOR_DATA_DESC_SET, &base_rate);

        if( result != mscl::MIP_ACK_OK )
            return fprintf(stderr, "Failed to get base rate: %s (%d)\n", mscl::MipCmdResult_toString(result), result), 1;

        // Set the message format to stream at 100 Hz.

        const uint16_t sample_rate = 100; // Hz
        const uint16_t decimation = base_rate / sample_rate;

        std::array<mscl::MipDescriptorRate, 3> descriptors = {{
            { mscl::MIP_DATA_DESC_SENSOR_ACCEL_SCALED, decimation },
            { mscl::MIP_DATA_DESC_SENSOR_GYRO_SCALED,  decimation },
            { mscl::MIP_DATA_DESC_SENSOR_MAG_SCALED,   decimation },
        }};

        result = mscl::write_mip_cmd_3dm_message_format(device.get(), mscl::MIP_SENSOR_DATA_DESC_SET, descriptors.size(), descriptors.data());

        if( result != mscl::MIP_ACK_OK )
            return fprintf(stderr, "Failed to set message format: %s (%d)\n", mscl::MipCmdResult_toString(result), result), 1;

        // Resume the device to ensure it's streaming.

        result = mscl::resume(device.get());
        if( result != mscl::MIP_ACK_OK )
            return fprintf(stderr, "Failed to resume device: %s (%d)\n", mscl::MipCmdResult_toString(result), result), 1;

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
