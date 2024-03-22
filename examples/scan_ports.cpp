
#include <libudev.h>

#include <string>
#include <thread>
#include <chrono>

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <csignal>


template<class T, T* (*Ref)(T*), T* (*Unref)(T*)>
class UdevHandle
{
public:
    UdevHandle(T* p) : m_ptr(p) {}
    ~UdevHandle() { if(m_ptr) { Unref(m_ptr); m_ptr=nullptr; } }

    UdevHandle(const UdevHandle& other) : m_ptr(other.m_ptr) { if(m_ptr) Ref(m_ptr); }
    UdevHandle& operator=(const UdevHandle& other) { m_ptr=other.m_ptr; if(m_ptr) Ref(m_ptr); }

    UdevHandle(UdevHandle&& other) { std::swap(m_ptr, other.m_ptr); }
    UdevHandle& operator=(UdevHandle&& other) { std::swap(m_ptr, other.m_ptr); }

    operator T*() { return m_ptr; }
    T* operator->() { return m_ptr; }

private:
    T* m_ptr = nullptr;
};


using UdevContext    = UdevHandle<udev, udev_ref, udev_unref>;
using UdevEnumerator = UdevHandle<udev_enumerate, udev_enumerate_ref, udev_enumerate_unref>;
using UdevDevice     = UdevHandle<udev_device, udev_device_ref, udev_device_unref>;
using UdevMonitor    = UdevHandle<udev_monitor, udev_monitor_ref, udev_monitor_unref>;

constexpr const char* const VENDOR  = "0483";
constexpr const char* const PRODUCT = "5740";


volatile std::sig_atomic_t stop_watching = false;

void handle_ctrl_c(int sig)
{
    stop_watching = true;
}


void watch_devices(UdevContext context)
{
    UdevMonitor monitor = udev_monitor_new_from_netlink(context, "udev");
    if(!monitor)
    {
        std::fprintf(stderr, "Failed to create udev monitor: %s\n", std::strerror(errno));
        return;
    }

    udev_monitor_filter_add_match_subsystem_devtype(monitor, "usb", NULL);

    udev_monitor_enable_receiving(monitor);

    while(!stop_watching)
    {
        UdevDevice device = udev_monitor_receive_device(monitor);
        if(device)
            std::printf("Device event: device=%s action=%s\n", udev_device_get_property_value(device, "DEVNAME"), udev_device_get_action(device));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::puts("Stopped.\n");
}


int main(int argc, const char* argv[])
{
    size_t num_devices = 0;

    UdevContext context(udev_new());
    if(!context)
    {
        std::fprintf(stderr, "Failed to create udev context: %s\n", std::strerror(errno));
        return 1;
    }

    UdevEnumerator usb_parent_enumerator(udev_enumerate_new(context));
    if(!usb_parent_enumerator)
    {
        std::fprintf(stderr, "Failed to create usb parent enumerator: %s\n", std::strerror(errno));
        return 1;
    }

    udev_enumerate_add_match_sysattr(usb_parent_enumerator, "idVendor",  VENDOR);
    udev_enumerate_add_match_sysattr(usb_parent_enumerator, "idProduct", PRODUCT);

    udev_enumerate_scan_devices(usb_parent_enumerator);

    for(udev_list_entry* entry = udev_enumerate_get_list_entry(usb_parent_enumerator); entry; entry = udev_list_entry_get_next(entry))
    {
        const char* device_name = udev_list_entry_get_name(entry);
        std::printf("Device %s\n", device_name);

        num_devices++;

        UdevDevice device = udev_device_new_from_syspath(context, device_name);
        if(!device)
        {
            std::fprintf(stderr, "Failed to create device: %s\n", std::strerror(errno));
            continue;
        }

        std::string serial;
        if(const char* c_str = udev_device_get_sysattr_value(device, "serial"))
            serial = c_str;
        else
        {
            std::fprintf(stderr, "Failed to get serial number: %s\n", std::strerror(errno));
            continue;
        }

        UdevEnumerator child_enumerator = udev_enumerate_new(context);
        if(!child_enumerator)
        {
            std::fprintf(stderr, "Failed to create child enumerator: %s\n", std::strerror(errno));
            continue;
        }

        udev_enumerate_add_match_subsystem(child_enumerator, "tty");
        udev_enumerate_add_match_parent(child_enumerator, device);

        udev_enumerate_scan_devices(child_enumerator);

        for(udev_list_entry* child_entry = udev_enumerate_get_list_entry(child_enumerator); child_entry; child_entry = udev_list_entry_get_next(child_entry))
        {
            const char* child_name = udev_list_entry_get_name(child_entry);
            std::printf("  Child %s\n", child_name);

            UdevDevice child = udev_device_new_from_syspath(context, child_name);
            if(!child)
            {
                std::fprintf(stderr, "Failed to get child device: %s\n", std::strerror(errno));
                continue;
            }

            std::printf("    Port name is %s\n", udev_device_get_property_value(child, "DEVNAME"));
        }
    }

    std::printf("Found %zu devices.\n", num_devices);

    if( std::signal(SIGTERM, handle_ctrl_c) != SIG_ERR )
        watch_devices(context);

    return 0;
}
