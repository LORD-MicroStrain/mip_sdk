#include "mip/utils/timestamp.hpp"

#include <stdexcept>

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

    void TimestampExperimental::increment(const TimestampExperimental &synced, const TimestampExperimental &old)
    {
        mip::Nanoseconds m_synced = synced.getTimestamp();
        mip::Nanoseconds m_old = old.getTimestamp();
        if (m_synced < m_old)
        {
            throw std::invalid_argument("Reference timestamp < old timestamp.");
        }
        
        m_timestamp += (m_synced - m_old);
    }

    Nanoseconds TimestampExperimental::getTimestamp() const
    {
        return m_timestamp;
    }

    Nanoseconds TimestampExperimental::getTimestampBase() const
    {
        return m_standard.convertToBase(m_timestamp);
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
        m_timestamp = convertFrom(from);
    }

    void TimestampExperimental::setWeek(Weeks week)
    {
        if (week < Weeks(0))         
        {
            throw std::invalid_argument("Week < 0.");
        }

        m_timestamp = week + getTimeOfWeek();
    }

    Nanoseconds TimestampExperimental::getTimeOfWeek()
    {
        return m_timestamp % Weeks(1);
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
} // namespace mip
