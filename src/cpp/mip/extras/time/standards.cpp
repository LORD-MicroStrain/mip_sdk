#include "mip/extras/time/standards.hpp"

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
}