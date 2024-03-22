
#include "port_manager_win.hpp"


namespace mip
{
namespace platform
{

PortManager_Windows::PortManager_Windows(HWND hwnd) : m_window(hwnd)
{
    // RegisterDeviceNotification
}

std::vector<PortManager::DeviceEntry> PortManager_Windows::listPorts()
{
    struct SetupTokens
    {
        GUID  guid;
        DWORD flags;
    };
    const SetupTokens setup_tokens[] = {
        { GUID_DEVCLASS_PORTS, DIGCF_PRESENT },
        { GUID_DEVCLASS_MODEM, DIGCF_PRESENT },
        { GUID_DEVINTERFACE_COMPORT, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE },
        { GUID_DEVINTERFACE_MODEM, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE },
    };

    std::vector<DeviceEntry> entries;

    for(auto token : setup_tokens)
    {
    }
}

}  // namespace platform
}  // namespace mip
