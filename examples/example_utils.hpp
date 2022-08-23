
#pragma once

#ifdef MIP_USE_SERIAL
    #include "mip/platform/serial_device_interface.hpp"
#endif
#ifdef MIP_USE_SOCKETS
    #include "mip/platform/tcp_device_interface.hpp"
#endif

#include <mip/mip_device.hpp>

#include <chrono>
#include <memory>
#include <exception>

mip::Timestamp getCurrentTimestamp();

std::unique_ptr<mip::DeviceInterface> openDeviceFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port);

std::unique_ptr<mip::DeviceInterface> handleCommonArgs(int argc, const char* argv[], int maxArgs=3);
int printCommonUsage(const char* argv[]);
