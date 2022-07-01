
#ifdef MSCL_USE_SERIAL
    #include "serial_mip_device.hpp"
#endif
#ifdef MSCL_USE_SOCKETS
    #include "tcp_mip_device.hpp"
#endif

#include <mip/mip_device.hpp>
#include <mip/definitions/commands_base.h>


#include <vector>
#include <memory>
#include <cstring>
#include <stdio.h>


#ifdef WIN32
    #define PORT_KEY "COM"
#else
    #define PORT_KEY "/dev/"
#endif



int usage(const char* argv[])
{
    fprintf(stderr, "Usage: %s <portname> <baudrate>\nUsage: %s <hostname> <port>", argv[0], argv[0]);
    return 1;
}


int main(int argc, const char* argv[])
{
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
    else if( argc != 3 )
        return usage(argv);

    std::unique_ptr<mscl::MipDeviceInterface> device;

    try
    {

        std::string port_or_hostname = argv[1];
        if(port_or_hostname.find(PORT_KEY) == std::string::npos)  // Not a serial port
        {
            uint32_t port = std::strtoul(argv[2], nullptr, 10);
            if( port < 1024 || port > 65535 )
            {
                fprintf(stderr, "Error: invalid port '%s'\n", argv[2]);
                return 1;
            }

            device.reset(new TcpMipDevice(port_or_hostname, port));
        }
        else  // Serial port
        {
            uint32_t baud = std::strtoul(argv[2], nullptr, 10);
            if( baud == 0 )
            {
                fprintf(stderr, "Error: invalid baudrate '%s'\n", argv[2]);
                return 1;
            }

            device.reset(new SerialMipDevice(port_or_hostname, baud));
        }

        mscl::MipBaseDeviceInfo device_info;

        mscl::MipCmdResult result = mscl::get_device_information(device.get(), &device_info);

        if( result == mscl::MIP_ACK_OK )
        {
            printf("Success:\n");

            auto print_info = [](const char* name, const char info[16])
            {
                char msg[17] = {0};
                std::strncpy(msg, info, 16);
                printf("  %s%s\n", name, msg);
            };

            print_info("Model name:       ", device_info.model_name);
            print_info("Model number:     ", device_info.model_number);
            print_info("Serial Number:    ", device_info.serial_number);
            print_info("Device Options:   ", device_info.device_options);
            print_info("Lot Number:       ", device_info.lot_number);

            printf(  "  Firmware version:           %d.%d.%d\n\n",
                (device_info.firmware_version / 1000),
                (device_info.firmware_version / 100) % 10,
                (device_info.firmware_version / 1)   % 100
            );
        }
        else
        {
            printf("Error: command completed with NACK: %s (%d)\n", mscl::MipCmdResult_toString(result), result);
        }
    }
    catch(const std::exception& ex)
    {
        fprintf(stderr, "Error: %s\n", ex.what());
        return 1;
    }

    return 0;
}
//
// namespace mscl
// {
// namespace C
// {
//
// MipCmdResult mipcmd_base_getDeviceInfo(struct C::MipInterfaceState* device, struct MipBaseDeviceInfo* info)
// {
//     uint8_t payload[MIP_FIELD_LENGTH_MAX];
//
//     size_t payloadLength = insert_MipCmd_Base_GetDeviceInfo(payload, sizeof(payload), 0, NULL);
//     assert(payloadLength <= sizeof(payload));
//
//     uint8_t responseLength;
//     MipCmdResult result = C::MipInterface_runCommandWithResponse(device, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, payload, payloadLength, MIP_REPLY_DESC_BASE_DEVICE_INFO, payload, &responseLength);
//     if( result == MIP_ACK_OK )
//     {
//         size_t used = extract_MipBaseDeviceInfo(payload, responseLength, 0, info);
//         if( used != responseLength )
//             result = MIP_STATUS_ERROR;
//     }
//
//     return result;
// }
//
// MipCmdResult mipcmd_base_getDeviceInfo(struct C::MipInterfaceState* device, const struct MipCmd_Base_GetDeviceInfo* cmd, struct MipCmd_Base_GetDeviceInfo_Response* response)
// {
//     uint8_t buffer[MIP_PACKET_LENGTH_MAX];
//
//     struct C::MipPacket packet;
//     MipPacket_create(&packet, buffer, sizeof(buffer), MIP_BASE_COMMAND_DESC_SET);
//
//     uint8_t* payload;
//     RemainingCount available = MipPacket_allocField(&packet, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, 0, &payload);
//
//     size_t used = insert_MipCmd_Base_GetDeviceInfo(payload, available, 0, cmd);
//     assert( used <= available );
//     MipPacket_reallocLastField(&packet, payload, used);
//
//     MipPacket_finalize(&packet);
//
//
//     struct C::MipPendingCmd pending;
//     C::MipPendingCmd_initWithResponse(&pending, MIP_BASE_COMMAND_DESC_SET, MIP_CMD_DESC_BASE_GET_DEVICE_INFO, MIP_REPLY_DESC_BASE_DEVICE_INFO, buffer, MIP_FIELD_PAYLOAD_LENGTH_MAX);
//
//     C::MipCmdQueue_enqueue( C::MipInterface_cmdQueue(device), &pending );
//
//     if( !C::MipInterface_sendToDevice(device, MipPacket_pointer(&packet), MipPacket_totalLength(&packet)) )
//     {
//         C::MipCmdQueue_dequeue(C::MipInterface_cmdQueue(device), &pending);
//         return MIP_STATUS_ERROR;
//     }
//
//     MipCmdResult result = C::MipInterface_waitForReply(device, &pending);
//
//     if( result == MIP_ACK_OK )
//     {
//         size_t responseLength = C::MipPendingCmd_responseLength(&pending);
//
//         used = extract_MipCmd_Base_GetDeviceInfo_Response( C::MipPendingCmd_response(&pending), responseLength, 0, response);
//
//         if( used!= responseLength )
//             return MIP_STATUS_ERROR;
//     }
//
//     return result;
// }
//
// }
// }
