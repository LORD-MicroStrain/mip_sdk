#pragma once

#include "port_manager.hpp"

#include <windows.h>


namespace mip
{
namespace platform
{

class PortManager_Windows : public PortManager
{
public:
    PortManager_Windows(HWND window);

    PortManager_Windows(const PortManager_Windows&) = delete;
    void operator=(const PortManager_Windows&) = delete;

    std::vector<DeviceEntry> listPorts() final;

private:
    HWND m_window = INVALID_HWND;
};

}  // namespace platform
}  // namespace mip
