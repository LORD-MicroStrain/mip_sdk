#include "mip/utils/timestamp.hpp"

#include <stdexcept>

namespace mip
{
    // TODO: Get Now and synchronize working.
    // TODO: Find a way to store the time standard.
    // TimestampExperimental TimestampExperimental::Now(const TimeStandard &standard)
    // {
    //     TimestampExperimental timestamp(standard);
    //     timestamp.synchronize();
    //     return timestamp;
    // }
    
    // void TimestampExperimental::synchronize()
    // {
    //     m_timestamp = standard.now();
    // }
} // namespace mip

    // TimestampExperimental::TimestampManager(const TimeStandard &standard)
    // {
    //     now(standard);
    // }

    // void TimestampExperimental::now(const TimeStandard &standard)
    // {
    //     m_timestamp = standard.now();
        // Seconds epoch_difference(0);

    // #if __APPLE__ || __linux__ || !_HAS_CXX20
    //     static constexpr int leap_seconds = 18; 

    //     switch (standard.id)
    //     {
    //     case TimeStandard::StandardId::UNIX:
    //         break;
    //     case TimeStandard::StandardId::GPS:
    //         epoch_difference = Seconds(315964800 - leap_seconds);
    //     default:
    //         throw std::invalid_argument("Invalid time standard.");
    //     }
    // #endif

        // m_timestamp = standard.timeSinceEpoch() - duration_cast<Nanoseconds>(epoch_difference);
    // }

    // void TimestampManager::synchronize(TimeStandard standard)
    // {
    //     // switch (standard)
    //     // {
    //     //     case TimeStandard::GPS:
    //     //         synchronize_();
    //     // }
    // }

    // void TimestampManager::synchronize_()
    // {
    // #if __APPLE__ || __linux__ || !_HAS_CXX20
    //     // Fallback GPS timestamp calculator for non-supported OSes.
    //     // NOTE: GPS time accounts for leap seconds.
    //     // NOTE: Unix time was created in 1970 while GPS time was created in 1980,
    //     //       so there is a difference between the times since epochs.
    //     static constexpr short leap = 18; 
    //     static constexpr int epoch_gap = 315964800; 
    //     auto now_unix = std::chrono::system_clock::now().time_since_epoch();
    //     m_timestamp = (now_unix - Seconds{epoch_gap}) + Seconds{leap};
    // #else
    //     // Auto GPS timestamp calculator (preferred). Not supported on all OSes.
    //     auto now_gps = std::chrono::gps_clock::now().time_since_epoch();
    //     m_timestamp = std::chrono::duration_cast<Nanoseconds>(now_gps);
    // #endif
    // }