
#include "mip_device.hpp"

namespace mscl {
namespace C {
extern "C" {

bool MipInterface_userPoll(struct MipInterfaceState* device)
{
    return static_cast<MipDeviceInterface*>(device)->poll();
}
bool MipInterface_userSendToDevice(struct MipInterfaceState* device, const uint8_t* data, size_t length)
{
    return static_cast<MipDeviceInterface*>(device)->sendToDevice(data, length);
}

} // extern "C"
} // namespace C
} // namespace mscl
