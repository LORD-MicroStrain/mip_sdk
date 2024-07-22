#pragma once

#include "mip/utils/time/durations.hpp"
#include "mip/utils/time/standards.hpp"

namespace mip
{
    /// Converts time duration from one time standard to another.
    /// 
    /// @throws std::logic_error If time < 0 after conversion (corresponding time duration
    ///         doesn't exist in the other standard).
    ///
    template<typename DurationOut = Nanoseconds, typename DurationIn>
    DurationOut convert(DurationIn time, const TimeStandard &to, const TimeStandard &from);

    /// Casts a time duration to the given arithmetic type.
    ///
    /// Uses a static_cast, so typical rounding rules will apply.
    ///
    /// @throws std::invalid_argument If time < 0 nanoseconds.
    ///
    template<typename T, typename DurationIn>
    T castTime(const DurationIn &time);


    /**************************************************************************************/
    /* Definitions for template declarations above.                                       */
    /**************************************************************************************/

    template<typename DurationOut, typename DurationIn>
    DurationOut convert(DurationIn time, const TimeStandard &to, const TimeStandard &from)
    {
        Nanoseconds converted = to.convertFromBase(from.convertToBase(std::chrono::duration_cast<Nanoseconds>(time)));
        if (converted < Nanoseconds(0))
        {
            throw std::logic_error("Converted time < 0. Add exception handling with desired response!");
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
