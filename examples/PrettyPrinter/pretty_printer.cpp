
#include "../example_utils.hpp"

#include "stringify.hpp"

#include <mip/metadata/mip_definitions.hpp>

#include <mip/definitions/commands_base.hpp>
#include <mip/metadata/definitions/data_sensor.hpp>
#include <mip/metadata/mip_all_definitions.hpp>

#include <csignal>
#include <thread>
#include <map>


volatile sig_atomic_t stop_flag = false;
mip::Timestamp last_receive_time = 0;

void signal_handler(int signum)
{
    (void)signum;

    stop_flag = true;
}


void handleField(void*, const mip::FieldView& field, mip::Timestamp timestamp)
{
    std::printf("%zu: %s\n", timestamp, formatField(field).c_str());

    last_receive_time = timestamp;
}


int main(int argc, const char* argv[])
{
    std::unique_ptr<ExampleUtils> utils;
    try {
        utils = handleCommonArgs(argc, argv);
    } catch(const std::underflow_error&) {
        return printCommonUsage(argv);
    } catch(const std::exception& ex) {
        fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }

    std::unique_ptr<mip::Interface>& device = utils->device;

    if(mip::CmdResult result = mip::commands_base::resume(*device); !result)
    {
        fprintf(stderr, "Error: Resume command failed: %d %s\n", result.value, result.name());
        return 1;
    }

    mip::DispatchHandler handler;
    device->registerFieldCallback<&handleField>(handler, 0x00, mip::INVALID_FIELD_DESCRIPTOR);

    mip::commands_base::BaseDeviceInfo info;
    mip::commands_base::getDeviceInfo(*device, &info);

    std::signal(SIGTERM, &signal_handler);

    while(!stop_flag)
    {
        device->update(100);
    }

    printf("Stopped.\n");

    return 0;
}

