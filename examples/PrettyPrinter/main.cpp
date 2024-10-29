
#include "../example_utils.hpp"

#include <mip/metadata/mip_all_definitions.hpp>
#include <mip/metadata/mip_format_bytes.hpp>
#include <mip/metadata/mip_formatter.hpp>

#ifdef MICROSTRAIN_ENABLE_SERIAL
#include <microstrain/connections/serial/serial_connection.hpp>
#endif
#ifdef MICROSTRAIN_ENABLE_TCP
#include <microstrain/connections/tcp/tcp_connection.hpp>
#endif

#include <iostream>
#include <memory>
#include <cstdio>
#include <cstring>


mip::metadata::BasicFormatter formatter(std::cout);
mip::metadata::FieldByteFormatter decoder(formatter);

void setup()
{
    handleCtrlC();
}

bool handlePacket(const mip::PacketView& packet, mip::Timestamp)
{
    decoder.formatPacket(packet, mip::metadata::definitions);
    std::cout << '\n';

    return !stop_flag;
}

int runFromFile(const char* filename)
{
    FILE* file = std::fopen(filename, "rb");
    if(!file)
    {
        auto err = errno;
        std::cerr << "Could not open file at '" << filename << "': " << std::strerror(err) << '\n';
        return 1;
    }

    uint8_t buffer[1024];
    mip::Parser parser(buffer, sizeof(buffer), 100);

    parser.setCallback<&handlePacket>();
    setup();

    while(!stop_flag)
    {
        uint8_t* ptr;
        size_t count = mip::C::mip_parser_get_write_ptr(&parser, &ptr);
        if( std::fread(ptr, 1, count, file) != count )
        {
            auto err = errno;
            std::cerr << "Error reading from file: " << std::strerror(err) << '\n';
            return 2;
        }

        mip::C::mip_parser_process_written(&parser, count, 0, MIPPARSER_UNLIMITED_PACKETS);
    }

    std::cout << "Stopped.\n";

    return 0;
}

#if defined MICROSTRAIN_ENABLE_SERIAL || defined MICROSTRAIN_ENABLE_TCP

int runFromDevice(int argc, const char* argv[])
{
    std::unique_ptr<ExampleUtils> utils;
    try {
        utils = handleCommonArgs(argc, argv);
    } catch(const std::underflow_error&) {
        return printCommonUsage(argv);
    } catch(const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }

    std::unique_ptr<mip::Interface>& device = utils->device;

    mip::DispatchHandler handler;
    mip::C::mip_interface_register_packet_callback(device.get(), &handler, 0x00, false,
        [](void*, const mip::C::mip_packet_view* packet, mip::Timestamp timestamp)
        {
            handlePacket(mip::PacketView(*packet), timestamp);
        },
        nullptr
    );

    setup();

    while(!stop_flag)
    {
        if(!device->update(100))
        {
            auto err = errno;
            std::cerr << "Error reading from device: " << std::strerror(err) << '\n';
            return 2;
        }
    }

    std::cout << "Stopped.\n";

    return 0;
}

#endif // MICROSTRAIN_ENABLE_SERIAL || MICROSTRAIN_ENABLE_TCP

int usage()
{
    std::cerr <<
        "Usage:\n"
        "    PrettyPrint <filename>\n"
        "        Prints the contents of a recorded binary file containing MIP packets.\n"
#ifdef MICROSTRAIN_ENABLE_SERIAL
        "    PrettyPrint <serial_port> <baudrate> [filename]\n"
        "        Connects to a device via serial at the specified baud.\n"
#endif
#ifdef MICROSTRAIN_ENABLE_TCP
        "    PrettyPrint <hostname> <port> [filename]\n"
        "        Connects to a device via TCP at the specified hostname and port.\n"
#endif
        "\n"
        "When connecting to a device, it is assumed that the device is already streaming.\n"
        "The optional 3rd argument 'filename' may be used to record raw data to a file.\n"
        "Use SIGTERM (Ctrl+C) to stop and exit the process.\n"
        "\n"
    ;

    return 1;
}

int main(int argc, const char* argv[])
{
    if(argc == 2)
        return runFromFile(argv[1]);
#if MICROSTRAIN_ENABLE_SERIAL || MICROSTRAIN_ENABLE_TCP
    else if(argc == 3 || argc == 4)
        return runFromDevice(argc, argv);
#endif
    else
        return usage();
}

