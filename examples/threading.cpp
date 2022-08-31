
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
volatile bool stop = false;

unsigned int display_progress()
{
    // Display progress.

    std::printf("Progress: [");

    unsigned int count = numSamples;

    float progress = count / float(maxSamples);

    unsigned int threshold = std::lround(progress * 50);

    unsigned int i=0;
    for(; i<threshold; i++)
        std::putchar('#');
    for(; i<50; i++)
        std::putchar(' ');

    std::printf("] %.0f%%\r", progress * 100);

    return count;
}

void packet_callback(void*, const mip::Packet& packet, mip::Timestamp timestamp)
{
    numSamples++;
}

void device_thread_loop(mip::DeviceInterface* device)
{
    while(!stop)
    {
        if( !device->update(false) )
            break;

        std::this_thread::yield();
    }
}

bool update_device(mip::DeviceInterface& device, bool blocking)
{
    if( !blocking )
        return device.defaultUpdate(blocking);

    // Optionally display progress while waiting for command replies.
    // Displaying it here makes it update more frequently.
    //display_progress();

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return true;
}

#define USE_THREADS 1

int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<ExampleUtils> utils = handleCommonArgs(argc, argv);
        std::shared_ptr<mip::DeviceInterface> device = utils->device;

        // Disable all streaming channels.
        mip::commands_base::setIdle(*device);
        mip::commands_3dm::writeDatastreamControl(*device, 0x00, false);

        // Register a sensor data packet callback.
        mip::DispatchHandler dispatchHandler;
        device->registerPacketCallback<&packet_callback>(dispatchHandler, mip::data_sensor::DESCRIPTOR_SET, false);

        // Set the message format to stream scaled accel at 1/100th the base rate (around a few Hz).
        mip::DescriptorRate descriptor{ mip::data_sensor::DATA_ACCEL_SCALED, 100 };
        mip::commands_3dm::writeMessageFormat(*device, mip::data_sensor::DESCRIPTOR_SET, 1, &descriptor);

#if USE_THREADS
        // Set the update function. Before this call, command replies are processed by the main thread.
        // After this, replies will be processed by the device thread.
        device->setUpdateFunction<&update_device>();

        // Start the device thread.
        std::thread deviceThread( &device_thread_loop, device.get() );
#endif // USE_THREADS

        // Enable streaming.
        mip::commands_3dm::writeDatastreamControl(*device, mip::data_sensor::DESCRIPTOR_SET, true);
        mip::commands_base::resume(*device);

        unsigned int count = 0;
        do
        {
            count = display_progress();

            // Ping the device a bunch (stress test).
            // If setUpdateFunction above is commented out, this can crash the program.
            for(unsigned int i=0; i<10; i++)
                mip::commands_base::ping(*device);

        } while(count < maxSamples);

        std::printf("\nDone!\n");

        // Return the device to idle.
        mip::commands_base::setIdle(*device);

        stop = true;
#if USE_THREADS
        deviceThread.join();
#endif
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