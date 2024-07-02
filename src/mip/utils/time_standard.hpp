#pragma once

namespace mip
{
    struct TimeStandard
    {
        virtual Nanoseconds now() const = 0;
        virtual Nanoseconds convertToBase(Nanoseconds time) const = 0;
        virtual Nanoseconds convertFromBase(Nanoseconds time) const = 0;
    };

    struct UnixTime: TimeStandard
    {
        static const UnixTime instance;
    };
    
    static const UnixTime &base_time = UnixTime::instance;
    
    struct GpsTime: TimeStandard
    {

    };
} // namespace mip