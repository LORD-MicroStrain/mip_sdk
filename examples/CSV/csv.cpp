
#include "../example_utils.hpp"

#include <mip/metadata/mip_definitions.hpp>

#include <mip/definitions/commands_base.hpp>

#include <csignal>
#include <thread>
#include <map>


volatile sig_atomic_t stop_flag = false;
mip::Timestamp last_receive_time = 0;

mip::metadata::Definitions mipdefs;

void signal_handler(int signum)
{
    stop_flag = true;
}


void handleField(void*, const mip::FieldView& field, mip::Timestamp timestamp)
{
    printf("%zu: field 0x%02X%02X\n", timestamp, field.descriptorSet(), field.fieldDescriptor());
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
    device->registerFieldCallback<&handleField>(handler, 0xFF, mip::INVALID_FIELD_DESCRIPTOR);

    std::signal(SIGTERM, &signal_handler);

    while(!stop_flag)
    {
        device->update(100);
    }

    printf("Stopped.\n");

    return 0;
}

