#include "mip/utils/time_standard.hpp"

namespace mip
{
    // std::chrono::nanoseconds TimeStandard::now()
    // {
    // #if __APPLE__ || __linux__ || !_HAS_CXX20
    //     return std::chrono::system_clock::now().time_since_epoch();
    // #else
    //     switch (standard_id)
    //     {
    //     case StandardId::UNIX:
    //         return std::chrono::system_clock::now().time_since_epoch();
    //     case StandardId::GPS:
    //         return std::chrono::gps_clock::now().time_since_epoch();
    //     default:
    //         throw std::invalid_argument("Invalid time standard.");
    //     }
    // #endif
    // }
}