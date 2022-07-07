
#pragma once

#ifdef MSCL_USE_SERIAL
    #include "serial_mip_device.hpp"
#endif
#ifdef MSCL_USE_SOCKETS
    #include "tcp_mip_device.hpp"
#endif

#include <memory>
#include <exception>

std::unique_ptr<mscl::MipDeviceInterface> openDeviceFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port);

std::unique_ptr<mscl::MipDeviceInterface> handleCommonArgs(int argc, const char* argv[], int maxArgs=3);
int printCommonUsage(const char* argv[]);
