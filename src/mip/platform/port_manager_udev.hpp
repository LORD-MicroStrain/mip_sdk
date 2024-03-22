#pragma once

#include "port_manager.hpp"

#include <libudev.h>

namespace mip
{
namespace platform
{

template<class T, T *(*Ref)(T *), T *(*Unref)(T *)>
class UdevHandle
{
public:
    UdevHandle(T *p = nullptr) : m_ptr(p) {}
    ~UdevHandle() { reset(); }

    void reset() { if (m_ptr) { Unref(m_ptr); m_ptr = nullptr; } }

    UdevHandle(const UdevHandle &other) : m_ptr(other.m_ptr) { if (m_ptr) Ref(m_ptr); }
    UdevHandle &operator=(const UdevHandle &other) { m_ptr = other.m_ptr; if (m_ptr) Ref(m_ptr); }

    UdevHandle& operator=(T* p) { reset(); m_ptr = p; }

    UdevHandle(UdevHandle &&other) { std::swap(m_ptr, other.m_ptr); }
    UdevHandle &operator=(UdevHandle &&other) { std::swap(m_ptr, other.m_ptr); }

    operator T *() { return m_ptr; }
    T *operator->() { return m_ptr; }

private:
    T *m_ptr = nullptr;
};


using UdevContext    = UdevHandle<udev, udev_ref, udev_unref>;
using UdevEnumerator = UdevHandle<udev_enumerate, udev_enumerate_ref, udev_enumerate_unref>;
using UdevDevice     = UdevHandle<udev_device, udev_device_ref, udev_device_unref>;
using UdevMonitor    = UdevHandle<udev_monitor, udev_monitor_ref, udev_monitor_unref>;


class PortManager_Udev : public PortManager
{
public:
    PortManager_Udev(const PortManager_Udev &) = delete;
    void operator=(const PortManager_Udev &) = delete;

    std::vector<DeviceEntry> listPorts() final;

    DeviceEntry poll() final;

    bool listen() final;
    void stopListening() final;

private:
    bool createContext();

private:
    UdevContext m_context;
    UdevMonitor m_monitor;
};

} // namespace platform

} // namespace mip
