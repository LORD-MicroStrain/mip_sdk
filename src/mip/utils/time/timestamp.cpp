#include "mip/utils/time/timestamp.hpp"

#include "mip/utils/time/misc.hpp"

namespace mip
{
    TimestampExperimental::TimestampExperimental(const mip::TimeStandard &standard) :
        m_standard(standard)
    {
        synchronize();
    }

    void TimestampExperimental::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    void TimestampExperimental::increment(const TimestampExperimental &reference, const TimestampExperimental &old)
    {
        mip::Nanoseconds m_synced = reference.getTimestamp();
        mip::Nanoseconds m_old = old.getTimestamp();
        if (m_synced < m_old)
        {
            throw std::invalid_argument("Reference timestamp < old timestamp.");
        }
        
        m_timestamp += (m_synced - m_old);
    }

    void TimestampExperimental::setTimestamp(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }

    void TimestampExperimental::setTimestamp(const TimestampExperimental &from)
    {
        m_timestamp = convert(from.getTimestamp(), m_standard, from.m_standard);
    }

    void TimestampExperimental::setWeek(Weeks week)
    {
        if (week < Weeks(0))         
        {
            throw std::invalid_argument("Week < 0.");
        }

        m_timestamp = week + getTimeOfWeek();
    }

    void TimestampExperimental::setTimeOfWeek(Nanoseconds time)
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

    Nanoseconds TimestampExperimental::getTimestamp() const
    {
        return m_timestamp;
    }

    Nanoseconds TimestampExperimental::getTimeOfWeek()
    {
        return m_timestamp % Weeks(1);
    }
} // namespace mip
