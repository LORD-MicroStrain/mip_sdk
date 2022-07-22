
#include "mip_device.hpp"

namespace mscl {
namespace C {
extern "C" {

bool mip_interface_user_update(struct mip_interface* device)
{
    return static_cast<MipDeviceInterface*>(device)->update();
}
bool mip_interface_user_send_to_device(struct mip_interface* device, const uint8_t* data, size_t length)
{
    return static_cast<MipDeviceInterface*>(device)->sendToDevice(data, length);
}

} // extern "C"
} // namespace C
} // namespace mscl
