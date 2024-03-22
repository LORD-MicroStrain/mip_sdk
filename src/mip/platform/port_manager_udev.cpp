
#include "port_manager_udev.hpp"

#include "../mip_logging.h"

#include <cstring>
#include <cerrno>


namespace mip
{
namespace platform
{

bool PortManager_Udev::createContext()
{
    m_context = udev_new();

    if(!m_context)
    {
        MIP_LOG_ERROR("PortMonitor: Could not create udev context: %s\n", std::strerror(errno));
        return false;
    }

    return true;
}

std::vector<PortManager_Udev::DeviceEntry> PortManager_Udev::listPorts()
{
    if(!m_context && !createContext())
    {
        //MIP_LOG_ERROR("PortMonitor::scan: No context!\n");
        return {};
    }

    std::vector<DeviceEntry> devices;

    UdevEnumerator enumerator = udev_enumerate_new(m_context);
    if(!enumerator)
    {
        MIP_LOG_ERROR("PortMonitor::scan: Failed to create enumerator: %s\n", std::strerror(errno));
        return {};
    }

    udev_enumerate_add_match_sysattr(enumerator, "idVendor",  "0483");
    udev_enumerate_add_match_sysattr(enumerator, "idProduct", "5740");

    udev_enumerate_scan_devices(enumerator);

    for(udev_list_entry* entry = udev_enumerate_get_list_entry(enumerator); entry; entry = udev_list_entry_get_next(entry))
    {
        const char* device_name = udev_list_entry_get_name(entry);

        UdevDevice device = udev_device_new_from_syspath(m_context, device_name);
        if(!device)
        {
            MIP_LOG_ERROR("PortMonitor::scan: Failed to create device '%s': %s\n", device_name, std::strerror(errno));
            continue;
        }

        const char* serial = udev_device_get_sysattr_value(device, "serial");
        MIP_LOG_DEBUG("PortMonitor::scan: Found top-level device '%s' with serial '%s'\n", device_name, serial);

        UdevEnumerator child_enumerator = udev_enumerate_new(m_context);
        if(!child_enumerator)
        {
            MIP_LOG_ERROR("PortMonitor::scan: Failed to create child enumerator: %s\n", std::strerror(errno));
            continue;
        }

        udev_enumerate_add_match_subsystem(child_enumerator, "tty");
        udev_enumerate_add_match_parent(child_enumerator, device);

        udev_enumerate_scan_devices(child_enumerator);

        for(udev_list_entry* child_entry = udev_enumerate_get_list_entry(child_enumerator); child_entry; child_entry = udev_list_entry_get_next(child_entry))
        {
            const char* child_name = udev_list_entry_get_name(child_entry);

            UdevDevice child = udev_device_new_from_syspath(m_context, child_name);
            if(!child)
            {
                MIP_LOG_ERROR("PortMonitor::scan: Failed to create child device '%s': %s\n", child_name, std::strerror(errno));
                continue;
            }

            const char* dev_name = udev_device_get_property_value(child, "DEVNAME");
            MIP_LOG_INFO("PortMonitor::scan: Found device '%s', port name '%s'\n", child_name, dev_name);

            devices.push_back({Action::NONE, serial, dev_name});
        }
    }

    return devices;
}

bool PortManager_Udev::listen()
{
    if(!m_context && !createContext())
        return false;

    m_monitor = udev_monitor_new_from_netlink(m_context, "udev");

    if(!m_monitor)
    {
        MIP_LOG_ERROR("PortMonitor::poll: Failed to create monitor: %s\n", std::strerror(errno));
        return false;
    }

    udev_monitor_filter_add_match_subsystem_devtype(m_monitor, "usb", NULL);
    udev_monitor_enable_receiving(m_monitor);
}

void PortManager_Udev::stopListening()
{
    if(m_monitor)
        m_monitor.reset();
}

PortManager_Udev::DeviceEntry PortManager_Udev::poll()
{
    if(!m_context && !createContext())
        return {Action::NONE};

    // Create and start the monitor if not already running.
    if(!m_monitor)
    {
        MIP_LOG_ERROR("PortMonitor::poll: Not listening (call listen())\n");
        return {Action::NONE};
    }

    UdevDevice device = udev_monitor_receive_device(m_monitor);

    if(!device)
    {
        MIP_LOG_TRACE("PortMonitor::poll: No changes\n");
        return {Action::NONE};
    }
    else
    {
        const char* dev_name = udev_device_get_property_value(device, "DEVNAME");
        const char* action_name = udev_device_get_action(device);
        const char* serial = udev_device_get_property_value(device, "serial");
        MIP_LOG_INFO("PortMonitor::poll: Action '%s': Device '%s' serial '%s'\n", action_name, dev_name, serial);

        // Possible action_names are as follows but they depend on the drivers.
        // "add" - device is added to the system
        // "remove" - device is removed from the system
        // "change" - device has changed (see event properties)
        // "online" - device is available for use
        // "offline" - device is no longer available
        // For USB devices specifically:
        // "bind" - device driver is bound to a device
        // "unbind" - device driver is unbound

        Action action = Action::NONE;
        if(std::strcmp(action_name, "bind") == 0)
            action = Action::CONNECTED;
        else if(std::strcmp(action_name, "unbind") == 0)
            action = Action::DISCONNECTED;

        return {action, serial, dev_name};
    }
}

} // namespace platform
} // namespace mip
