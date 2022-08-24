
#include "example_utils.hpp"

#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/data_sensor.hpp>

#include <thread>
#include <chrono>
#include <cmath>
#include <cstdio>

const unsigned int maxSamples = 50;
volatile unsigned int numSamples = 0;

void packet_callback(void*, const mip::Packet& packet, mip::Timestamp timestamp)
{
    numSamples++;
}

void device_thread_loop(mip::DeviceInterface* device)
{
    while(numSamples < maxSamples)
    {
        if( !device->update() )
            break;

        std::this_thread::yield();
    }
}

int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<mip::DeviceInterface> device = handleCommonArgs(argc, argv);

        // Disable all streaming channels.
        mip::commands_base::setIdle(*device);
        mip::commands_3dm::writeDatastreamControl(*device, 0x00, false);

        // Register a sensor data packet callback.
        mip::DispatchHandler dispatchHandler;
        device->registerPacketCallback<&packet_callback>(dispatchHandler, mip::data_sensor::DESCRIPTOR_SET, false);

        // Set the message format to stream scaled accel at 1/100th the base rate (around a few Hz).
        mip::DescriptorRate descriptor{ mip::data_sensor::DATA_ACCEL_SCALED, 100 };
        mip::commands_3dm::writeMessageFormat(*device, mip::data_sensor::DESCRIPTOR_SET, 1, &descriptor);

        // Enable streaming.
        mip::commands_3dm::writeDatastreamControl(*device, mip::data_sensor::DESCRIPTOR_SET, true);
        mip::commands_base::resume(*device);

        // Start the updater thread.
        std::thread deviceThread( &device_thread_loop, device.get() );

        unsigned int count = 0;
        do
        {
            // Update the display every ~10 ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Display progress.

            std::printf("Progress: [");

            count = numSamples;

            float progress = count / float(maxSamples);

            unsigned int threshold = std::lround(progress * 50);

            unsigned int i=0;
            for(; i<threshold; i++)
                std::putchar('#');
            for(; i<50; i++)
                std::putchar(' ');

            std::printf("] %.0f%%\r", progress * 100);

            // Ping the device a bunch (stress test to trigger race condition).
            // This can crash the program due to servicing two update calls at once.
            for(unsigned int i=0; i<10; i++)
                mip::commands_base::ping(*device);

        } while(count < maxSamples);

        std::printf("\nDone!\n");

        deviceThread.join();
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