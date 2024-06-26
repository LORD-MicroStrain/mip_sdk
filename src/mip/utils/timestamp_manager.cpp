#include "mip/utils/timestamp_manager.hpp"

namespace mip
{
    Seconds TimestampManager::epochDifference(TimeStandard standard)
    {
    #if __APPLE__ || __linux__ || !_HAS_CXX20
        static constexpr int leap_seconds = 18; 

        int unix_epoch_difference = 0;
        switch (id)
        {
        case StandardId::UNIX:
            break;
        case StandardId::GPS:
            unix_epoch_difference = 315964800 - leap_seconds;
            break;
        }

        epoch_difference = mip::Seconds(unix_epoch_difference - leap_seconds); 
    #else
        epoch_difference = mip::Seconds(0);
    #endif
    }
    
    Nanoseconds TimestampManager::timeSinceEpoch(TimeStandard standard)
    {
        // TODO: Add standard stuff.
        return Nanoseconds(system_clock::now().time_since_epoch());

    }
    
    TimestampManager::TimestampManager(long long nanoseconds_since_epoch)
    {
        if (nanoseconds_since_epoch > 0)
        {
            m_timestamp = Nanoseconds(nanoseconds_since_epoch);
        }
    }
    
    TimestampManager::TimestampManager(TimeStandard standard)
    {
        synchronize(standard);
    }

    void TimestampManager::synchronize(TimeStandard standard)
    {
        m_timestamp = duration_cast<Nanoseconds>(timeSinceEpoch(standard) - epochDifference(standard));
    }

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
}// namespace mip
