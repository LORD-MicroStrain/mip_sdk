#pragma once

namespace mip
{
    // TODO: Move outside of class into separate utility.
    // throws logic error
    template<class DurationOut = Nanoseconds>
    DurationOut convertFrom(const TimestampExperimental &reference);

    // TODO: Move outside of class into separate utility.
    // TODO: Update documentation.
    /// Casts a timestamp duration to the given arithmetic type.
    template<typename T, typename DurationIn>
    T castTime(const DurationIn &time);


    /**************************************************************************************/
    /* Definitions for template declarations above.
    /**************************************************************************************/

    template<class DurationOut>
    inline DurationOut TimestampExperimental::convertFrom(const TimestampExperimental &reference)
    {
        Nanoseconds converted = m_standard.convertFromBase(reference.getTimestampBaseStandard());
        if (converted < Nanoseconds(0))
        {
            throw std::logic_error("Converted timestamp < 0. Add exception handling with desired response!");
        }
        
        return std::chrono::duration_cast<DurationOut>(converted);
    }

    template<typename T, typename DurationIn>
    T TimestampExperimental::castTime(const DurationIn &time)
    {
        if (time < mip::Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        return static_cast<T>(time.count());
    }
} // namespace mip
