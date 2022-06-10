
#include <stdio.h>

#include <mip/definitions/commands_base.h>
#include <mip/mip.hpp>
#include <serial/serial.h>


int usage(const char* argv[])
{
    fprintf(stderr, "Usage: %s <port> <baudrate>\n", argv[0]);
    return 1;
}


int main(int argc, const char* argv[])
{
    uint32_t baud = 0;

    if( argc == 1 )
    {
        printf("Available serial ports:\n");
        std::vector<serial::PortInfo> ports = serial::list_ports();

        for(const serial::PortInfo& port : ports)
        {
            printf("  %s %s %s\n", port.port.c_str(), port.description.c_str(), port.hardware_id.c_str());
        }
        return 0;
    }
    else if( argc == 3 )
    {
        baud = std::strtoul(argv[2], nullptr, 10);
        if( baud == 0 )
        {
            fprintf(stderr, "Error: invalid baud rate '%s'\n", argv[2]);
            return 1;
        }
    }
    else
    {
        return usage(argv);
    }

    try
    {
        serial::Serial port(argv[1], baud, serial::Timeout::simpleTimeout(10));

#define METHOD 1

#if METHOD == 1

        uint8_t buffer[MIP_PACKET_LENGTH_MAX];

        mscl::MipPacket packet(buffer, sizeof(buffer), mscl::MIP_BASE_COMMAND_DESC_SET);

        uint8_t* payload;
        mscl::RemainingCount available = packet.allocField(mscl::MIP_CMD_DESC_BASE_PING, 0, &payload);

        mscl::MipCmd_Base_Ping ping;
        mscl::insert_MipCmd_Base_Ping(payload, available, 0, &ping);

        packet.finalize();

        printf("Send bytes [");
        for(size_t i=0; i<packet.totalLength(); i++)
            printf("%02X", packet.pointer()[i]);
        printf("]\n");

        port.write(packet.pointer(), packet.totalLength());

        size_t read = port.read(buffer, sizeof(buffer));

        printf("Received bytes [");
        for(size_t i=0; i<read; i++)
            printf("%02X", buffer[i]);
        printf("]\n");
#endif
    }
    catch(const std::exception& ex)
    {
        fprintf(stderr, "Could not open serial port: %s", ex.what());
        return 1;
    }

    return 0;
}
