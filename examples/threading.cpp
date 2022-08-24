
#include "example_utils.hpp"

#include <thread>
#include <cmath>
#include <cstdio>

const unsigned int maxSamples = 50;
volatile unsigned int numSamples = 0;

void device_thread_loop(mip::DeviceInterface* device)
{
    while(numSamples < maxSamples)
    {
        if( !device->update() )
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        numSamples++;
    }
}

int main(int argc, const char* argv[])
{
    try
    {
        std::unique_ptr<mip::DeviceInterface> device = handleCommonArgs(argc, argv);

        std::thread deviceThread( &device_thread_loop, device.get() );

        unsigned int count = 0;
        do
        {
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