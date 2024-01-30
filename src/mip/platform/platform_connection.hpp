#pragma once

#include "mip/mip_device.hpp"

#include <memory>
#include <string_view>


namespace mip
{
    namespace platform
    {
        std::unique_ptr<mip::Connection> createConnectionFromInterfaceName(std::string_view interface_name, uint32_t parameter);

        bool isNetworkInterfaceName(std::string_view interface_name);
        bool isSerialInterfaceName(std::string_view interface_name);

    } // namespace platform
} // namespace mip
