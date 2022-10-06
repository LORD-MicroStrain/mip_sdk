
#include "mip_device.hpp"
#include "mip_logging.h"

namespace mip {
namespace C {
extern "C" {

bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, timestamp_type* timestamp_out)
{
    mip::Logging::info(device, "Hello from recv '%s'\n", "foo");
    return static_cast<DeviceInterface*>(device)->recvFromDevice(buffer, max_length, out_length, timestamp_out);
}

bool mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length)
{
    return static_cast<DeviceInterface*>(device)->sendToDevice(data, length);
}

} // extern "C"
} // namespace C
} // namespace mip
