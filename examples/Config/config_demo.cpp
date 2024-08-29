
#include "config.hpp"

#include "example_utils.hpp"

#include <mip/definitions/commands_base.hpp>
#include <mip/definitions/commands_3dm.hpp>

#include <cstdio>
#include <sstream>

using namespace mip;


bool printResults(CompositeResult& results)
{
    if(results.allSuccessful())
    {
        std::printf("Success!\n");
        return true;
    }
    else if(results.allFailed())
        std::printf("All commands failed, please check the device.\n");
    else //if(result.anyFailed())
    {
        std::printf("One or more settings failed! Details:\n");
        for(auto& result : results)
        {
            auto descriptor = result.descriptor.descriptor();
            auto instance = result.descriptor.index();

            std::printf(
                "  0x%02X%02X-%d: %d %s\n",
                descriptor.descriptorSet, descriptor.fieldDescriptor, instance,
                result.result.value, result.result.name()
            );
        }
    }

    return false;
}

void printConfig(Config& config)
{
    std::printf("All settings: \n");

    for(const std::unique_ptr<SettingBase>& setting : config)
    {
        std::putchar(' ');
        std::putchar(' ');
        setting->print();
    }

    std::putchar('\n');
}


int main(int argc, const char* argv[])
{
    std::unique_ptr<ExampleUtils> utils;
    try {
        utils = handleCommonArgs(argc, argv);
    } catch(const std::underflow_error&) {
        return printCommonUsage(argv);
    } catch(const std::exception& ex) {
        std::fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }

    std::unique_ptr<mip::Interface>& device = utils->device;

    if(!commands_base::ping(*device))
    {
        std::fprintf(stderr, "Failed to ping device!\n");
        return 1;
    }

    Config config(*device);

    config.registerSetting<commands_base::CommSpeed>(0x01, uint8_t(0x01));
    config.registerSetting<commands_3dm::PpsSource>();

    commands_base::BaseDeviceInfo devInfo;
    commands_base::getDeviceInfo(*device, &devInfo);

    std::printf("Reading all settings...\n");
    CompositeResult results = config.readAll();
    if( !printResults(results) )
        return 2;

    printConfig(config);

    std::printf("Reset and read all settings...\n");
    results = config.resetAll();
    if( !printResults(results) )
        return 2;

    printConfig(config);

    return 0;
}
