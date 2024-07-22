#include "mip/utils/time/standards.hpp"

namespace mip
{
    Nanoseconds UnixTime::now() const
    {
        return std::chrono::duration_cast<Nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
    }

    Nanoseconds UnixTime::convertToBase(Nanoseconds time) const
    {
        return time;         
    }

    Nanoseconds UnixTime::convertFromBase(Nanoseconds time) const
    {
        return time;      
    }

    Nanoseconds GpsTime::now() const
    {
        Nanoseconds now = std::chrono::duration_cast<Nanoseconds>(std::chrono::system_clock::now().time_since_epoch());

        return convertFromBase(now);
    }

    Nanoseconds GpsTime::convertToBase(Nanoseconds time) const
    {
        return time + epoch_difference;
    }

    Nanoseconds GpsTime::convertFromBase(Nanoseconds time) const
    {
        return time - epoch_difference;
    }

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
}