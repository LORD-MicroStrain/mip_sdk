
#include <mip/mip_device.hpp>
#include <mip/platform/serial_connection.hpp>
#include <mip/platform/tcp_connection.hpp>

#include <mip/definitions/commands_base.hpp>

#include <pybind11/pybind11.h>

#ifdef WIN32
#define SERIAL_KEY "COM"
#else
#define SERIAL_KEY "/dev/tty"
#endif


namespace py = pybind11;


class Device : public mip::DeviceInterface
{
public:
    Device(mip::Timeout baseTimeout = 100) :
        mip::DeviceInterface(m_buffer, sizeof(m_buffer), 100, baseTimeout)
    {
    }

    Device(std::shared_ptr<mip::Connection> connection, mip::Timeout baseTimeout=100) :
        mip::DeviceInterface(connection.get(), m_buffer, sizeof(m_buffer), 100, baseTimeout)
    {
        m_connection = connection;
    }

    std::shared_ptr<mip::Connection> connection() const { return m_connection; }

private:
    uint8_t m_buffer[4096];
    std::shared_ptr<mip::Connection> m_connection;
};

Device* connect(const std::string& interface, uint32_t parameter)
{
    std::shared_ptr<mip::Connection> connection;
    mip::Timeout replyTimeout;

    if(interface.find(SERIAL_KEY) == 0)
    {
        connection = std::make_shared<mip::platform::SerialConnection>(interface, parameter);
        replyTimeout = 100;
    }
    else
    {
        connection = std::make_shared<mip::platform::TcpConnection>(interface, parameter);
        replyTimeout = 2000;
    }

    connection->connect();

    std::unique_ptr<Device> device(new Device(connection, replyTimeout));

    return device.release();
}

// Result with descriptor
struct FullResult : public mip::CmdResult
{
    mip::CompositeDescriptor descriptor;

    template<class Cmd>
    FullResult(mip::TypedResult<Cmd> result) : mip::CmdResult(result), descriptor(result.descriptor()) {}
};


PYBIND11_MODULE(mip, m)
{
    m.doc() = "MIP SDK";

    py::class_<mip::Connection, std::shared_ptr<mip::Connection>>(m, "Connection")
        .def("send_to_device", &mip::Connection::sendToDevice)
        .def("recv_from_device", &mip::Connection::recvFromDevice)
        .def_property_readonly("is_open", &mip::Connection::isConnected)
        .def("open", &mip::Connection::connect)
        .def("close", &mip::Connection::disconnect)
        .def_property_readonly("type", &mip::Connection::type)
        .def("__str__", [](mip::Connection* conn){ return std::string(conn->interfaceName()) + "/" + std::to_string(conn->parameter()); })
    ;

    // mip.connect("/dev/ttyACM0", 115200)
    m.def("connect", &connect);

    py::class_<Device>(m, "Device")
        .def(py::init<std::shared_ptr<mip::Connection>, uint32_t>())
        .def_property_readonly("connection", &Device::connection)
        .def_property_readonly("is_connected", [](Device& device){ return device.connection() && device.connection()->isConnected(); })
        //.def_property_readonly("device_info", &Device::deviceInfo)
    ;

    py::class_<FullResult>(m, "Result")
        //.def(py::init<mip::CmdResult, mip::CompositeDescriptor>())
        .def_property_readonly("code", [](FullResult& result){ return int(result.value); })
        .def_property_readonly("name", &FullResult::name)
        .def_property_readonly("command", [](FullResult& result){ return py::make_tuple(result.descriptor.descriptorSet, result.descriptor.fieldDescriptor); })
        .def_property_readonly("is_reply", &FullResult::isReplyCode)
        .def_property_readonly("is_status", &FullResult::isStatusCode)
        .def("__bool__", [](FullResult& result){ return !!result; })
    ;

    auto base = m.def_submodule("commands_base", "Base commands");
    base.def("ping",   [](Device& device)->FullResult{ return mip::commands_base::ping(device); });
    base.def("idle",   [](Device& device)->FullResult{ return mip::commands_base::setIdle(device); });
    base.def("resume", [](Device& device)->FullResult{ return mip::commands_base::resume(device); });
}

