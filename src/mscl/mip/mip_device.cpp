
#include "mip_device.hpp"

namespace mip {
namespace C {
extern "C" {

bool mip_interface_user_update(struct mip_interface* device)
{
    return static_cast<DeviceInterface*>(device)->update();
}
bool mip_interface_user_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length)
{
    return static_cast<DeviceInterface*>(device)->sendToDevice(data, length);
}

} // extern "C"
} // namespace C
} // namespace mip
