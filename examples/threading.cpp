
/////////////////////////////////////////////////////////////////////////////
//
// threading.cpp
//
// C++ example program to print device information from any mip-enabled MicroStrain device.
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN BY HBK SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

#include "example_utils.hpp"

#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_3dm.hpp>
#include <mip/definitions/data_sensor.hpp>

#include <thread>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <algorithm>

const unsigned int maxSamples = 50;
volatile unsigned int numSamples = 0;
volatile bool stop = false;

unsigned int display_progress()
{
    // Display progress.

    std::printf("Progress: [");

    unsigned int count = numSamples;

    // Compute progress as a fraction from 0 to 1 (may exceed 100% if some extra packets get through).
    float progress = std::min(count / float(maxSamples), 1.0f);

    unsigned int threshold = std::lround(progress * 50);

    unsigned int i=0;
    for(; i<threshold; i++)
        std::putchar('#');
    for(; i<50; i++)
        std::putchar(' ');

    std::printf("] %.0f%%\r", progress * 100);
    std::fflush(stdout);

    return count;
}

void packet_callback(void*, const mip::PacketView&, mip::Timestamp)
{
    numSamples = numSamples + 1;
}

void device_thread_loop(mip::Interface* device)
{
    while(!stop)
    {
        if( !device->update(0) )
        {
            device->cmdQueue().clear();  // Avoid deadlocks if the socket is closed.
            break;
        }

        std::this_thread::yield();
    }
}

bool update_device(mip::Interface& device, mip::Timeout wait_time, bool from_cmd)
{
    // Do normal updates only if called from a command handler.
    // This separates the main thread from the data collection thread.
    if( !from_cmd )
        return device.defaultUpdate(wait_time, true);

    // Optionally display progress while waiting for command replies.
    // Displaying it here instead makes it update more frequently.
    //display_progress();

    // Sleep for a bit to save power. Note that waiting too long
    // in here can extend command timeout times.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Avoid failing the update function as long as the other thread is still running.
    // Doing so may cause a race condition (see comments in mip_interface_wait_for_reply).
    return true;
}

#define USE_THREADS 1

int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<ExampleUtils> utils = handleCommonArgs(argc, argv);
        std::unique_ptr<mip::Interface>& device = utils->device;

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
        device->setUpdateFunctionFree<&update_device>();

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
            // This attempts to trigger race conditions by having the main thread
            // send pings while the other thread tries to collect data.
            // Try commenting out the setUpdateFunction call
            // above and notice the erratic or errant behavior.
            // Note that only one thread at a time can safely send commands.
            for(unsigned int i=0; i<100; i++)
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
    catch(const std::underflow_error&)
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