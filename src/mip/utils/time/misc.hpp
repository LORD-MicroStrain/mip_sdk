#pragma once

#include "mip/utils/time/durations.hpp"
#include "mip/utils/time/standards.hpp"

// TODO: Add/update documentation.
namespace mip
{
    // throws logic error
    template<class DurationIn, class DurationOut = Nanoseconds>
    DurationOut convert(DurationIn time, const TimeStandard &to, const TimeStandard &from);

    template<class DurationOut = Nanoseconds>
    DurationOut convert(Nanoseconds time, const TimeStandard &to, const TimeStandard &from);

    // TODO: Update documentation.
    /// Casts a timestamp duration to the given arithmetic type.
    template<typename T, typename DurationIn>
    T castTime(const DurationIn &time);


    /**************************************************************************************/
    /* Definitions for template declarations above.                                       */
    /**************************************************************************************/

    template<class DurationIn, class DurationOut>
    DurationOut convert(DurationIn time, const TimeStandard &to, const TimeStandard &from)
    {
        return convert(std::chrono::duration_cast<Nanoseconds>(time), to, from);
    }

    template<class DurationOut>
    DurationOut convert(Nanoseconds time, const TimeStandard &to, const TimeStandard &from)
    {
        Nanoseconds converted = to.convertFromBase(from.convertToBase(time));
        if (converted < Nanoseconds(0))
        {
            throw std::logic_error("Converted timestamp < 0. Add exception handling with desired response!");
        }
        
        return std::chrono::duration_cast<DurationOut>(converted);
        
    }

    template<typename T, typename DurationIn>
    T castTime(const DurationIn &time)
    {
        if (time < mip::Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        return static_cast<T>(time.count());
    }
} // namespace mip
