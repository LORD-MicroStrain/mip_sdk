#include "mip/utils/timestamp.hpp"

#include <stdexcept>

namespace mip
{
    template<typename Standard>
    inline TimestampExperimental<Standard>::TimestampExperimental() :
        m_standard(Standard())
    {
        synchronize();
    }

    template<typename Standard>
    template<typename DurationIn> 
    inline TimestampExperimental<Standard>::TimestampExperimental(DurationIn time) :
        m_standard(Standard())
    {
        if (time < mip::Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }

    template<typename Standard>
    inline void TimestampExperimental<Standard>::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    template<typename Standard>
    inline void TimestampExperimental<Standard>::increment(const TimestampExperimental &reference_synced, const TimestampExperimental &reference_old)
    {
        mip::Nanoseconds m_synced = reference_synced.getTimestamp();
        mip::Nanoseconds m_old = reference_old.getTimestamp();
        if (m_synced < m_old)
        {
            throw std::invalid_argument("Reference timestamp < old timestamp.");
        }
        
        m_timestamp += (m_synced - m_old);
    }

    template<typename Standard>
    template<typename DurationOut> 
    inline DurationOut TimestampExperimental<Standard>::getTimestamp() const
    {
        return std::chrono::duration_cast<DurationOut>(getTimestamp());
    }

    template<typename Standard>
    inline Nanoseconds TimestampExperimental<Standard>::getTimestamp() const
    {
        return m_timestamp;
    }

    template<typename Standard>
    template<typename DurationIn>
    inline void TimestampExperimental<Standard>::setTimestamp(DurationIn time)
    {
        setTimestamp(std::chrono::duration_cast<Nanoseconds>(time));
    }
    
    template<typename Standard>
    inline void TimestampExperimental<Standard>::setTimestamp(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }

    template<typename Standard>
    inline void TimestampExperimental<Standard>::setWeek(Weeks week)
    {
        if (week < Weeks(0))         
        {
            throw std::invalid_argument("Week < 0.");
        }

        m_timestamp = week + getTimeOfWeek();
    }

    template<typename Standard>
    template<typename DurationOut>
    inline DurationOut TimestampExperimental<Standard>::getTimeOfWeek()
    {
        if (DurationOut(1) >= Weeks(1))
        {
            throw std::invalid_argument("Duration >= one week.");            
        }

        return std::chrono::duration_cast<DurationOut>(getTimeOfWeek());
    }

    template<typename Standard>
    inline Nanoseconds TimestampExperimental<Standard>::getTimeOfWeek()
    {
        return m_timestamp % Weeks(1);
    }

    template<typename Standard>
    template<typename DurationIn>
    inline void TimestampExperimental<Standard>::setTimeOfWeek(DurationIn time)
    {
        setTimeOfWeek(std::chrono::duration_cast<Nanoseconds>(time));
    }
    
    template<typename Standard>
    inline void TimestampExperimental<Standard>::setTimeOfWeek(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time of week < 0.");
        }
        if (time >= Weeks(1))
        {
            throw std::invalid_argument("Time of week >= one week.");
        }

        m_timestamp = std::chrono::duration_cast<Weeks>(m_timestamp) + time;
    }

    template<typename Standard>
    template<typename DurationElapsed>
    inline bool TimestampExperimental<Standard>::timeElapsed(const TimestampExperimental &reference)
    {
        const Nanoseconds m_reference = reference.getTimestamp();
        if (m_timestamp < m_reference)
        {
            throw std::invalid_argument("Timestamp < reference timestamp.");
        }
        
        return m_timestamp - m_reference >= DurationElapsed(1);
    }

    template<typename Standard>
    template<typename DurationChanged>
    inline bool TimestampExperimental<Standard>::timeChanged(const TimestampExperimental &reference)
    {
        const Nanoseconds m_reference = reference.getTimestamp();
        if (m_timestamp < m_reference)
        {
            throw std::invalid_argument("Timestamp < reference timestamp.");
        }

        return std::chrono::duration_cast<DurationChanged>(m_timestamp) > std::chrono::duration_cast<DurationChanged>(m_reference);
    }

    template<typename Standard>
    template<typename T, typename DurationIn>
    T TimestampExperimental<Standard>::castTime(const DurationIn &time)
    {
        if (time < mip::Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }
        return static_cast<T>(time.count());
    }
} // namespace mip