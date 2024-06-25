
#include "../example_utils.hpp"

#include <mip/metadata/mip_definitions.hpp>

#include <mip/definitions/commands_base.hpp>
#include <mip/metadata/definitions/data_sensor.hpp>

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
    const mip::metadata::FieldInfo* info = mipdefs.findField(field.descriptor());
    if(info)
    {
        printf("%zu: %s(", timestamp, info->name);
        size_t i = 0;
        for(const auto& param : info->parameters)
        {
            if(i>0)
                fputs(", ", stdout);

            switch(param.type.type)
            {
            case mip::metadata::Type::BOOL:   printf("%d",  microstrain::read<bool    >(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::U8:     printf("%u",  microstrain::read<uint8_t >(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::S8:     printf("%d",  microstrain::read< int8_t >(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::U16:    printf("%u",  microstrain::read<uint16_t>(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::S16:    printf("%d",  microstrain::read< int16_t>(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::U32:    printf("%u",  microstrain::read<uint32_t>(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::S32:    printf("%d",  microstrain::read< int32_t>(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::U64:    printf("%lu", microstrain::read<uint64_t>(field.payload()+param.byte_offset)); break;
            case mip::metadata::Type::S64:    printf("%ld", microstrain::read< int64_t>(field.payload()+param.byte_offset)); break;
            case mip::metadata::Type::FLOAT:  printf("%f",  microstrain::read<float   >(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::DOUBLE: printf("%f",  microstrain::read<double  >(field.payload()+param.byte_offset));  break;
            case mip::metadata::Type::STRUCT:   printf("{}"); break;
            case mip::metadata::Type::ENUM:     printf("%s", "enum"); break;
            case mip::metadata::Type::BITFIELD: printf("%s", "bits"); break;
            case mip::metadata::Type::UNION:    printf("%s", "union"); break;
            default: printf("?"); break;
            }
            ++i;
        }
        fputs(")\n", stdout);
    }
    else
        printf("%zu: field 0x%02X%02X\n", timestamp, field.descriptorSet(), field.fieldDescriptor());

    last_receive_time = timestamp;
}


int main(int argc, const char* argv[])
{
    mipdefs.registerDefinitions(mip::metadata::ALL_SENSOR_DATA);

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

