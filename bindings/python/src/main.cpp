
#include "mip_python.hpp"

#include <mip/definitions/commands_base.hpp>

#include <pybind11/pybind11.h>



namespace py = pybind11;


extern Device* connect(const std::string& interface, uint32_t parameter);

extern void bind_commands_base(py::module_&);
extern void bind_commands_3dm(py::module_&);


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
        .def("__str__", [](Device& device){ return device.connection()->interfaceName(); })  // Todo null check
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

    auto cmd_3dm = m.def_submodule("commands_3dm", "3DM Commands");
    bind_commands_3dm(cmd_3dm);
}

