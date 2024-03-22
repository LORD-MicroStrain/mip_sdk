#pragma once

#include <string>
#include <vector>


namespace mip
{
namespace platform
{

class PortManager
{
public:
    enum class Action
    {
        NONE = 0,
        CONNECTED,
        DISCONNECTED,
    };

    struct DeviceEntry
    {
        Action      action = Action::NONE;
        std::string serial;
        std::string path;
    };

    virtual std::vector<DeviceEntry> listPorts() = 0;

    virtual bool listen() = 0;
    virtual DeviceEntry poll() = 0;
    virtual void stopListening() = 0;
};

} // namespace platform
} // namespace mip
