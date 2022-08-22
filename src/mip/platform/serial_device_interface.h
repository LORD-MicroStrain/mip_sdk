#pragma once

// Handle type depending on OS
#ifdef _WIN32
#include <windows.h>
typedef HANDLE serial_device_interface_handle;
#else
typedef int serial_device_interface_handle;
#endif

struct serial_device_interface
{
    bool is_open;
    serial_device_interface_handle handle;
};

bool serial_device_interface_open(struct serial_device_interface* port, const char* port_str, int baudrate);
bool serial_device_interface_close(struct serial_device_interface* port);
bool serial_device_interface_update(struct serial_device_interface* port, struct mip_interface* device);
bool serial_device_interface_send_to_device(struct serial_device_interface* port, const uint8_t* data, size_t num_bytes);