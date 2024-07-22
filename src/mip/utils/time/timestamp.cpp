#include "mip/utils/time/timestamp.hpp"

#include "mip/utils/time/misc.hpp"

namespace mip
{
    Timestamp::Timestamp(const mip::TimeStandard &standard) :
        m_standard(standard)
    {
        synchronize();
    }

    void Timestamp::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    void Timestamp::increment(const Timestamp &reference_new, const Timestamp &reference_old)
    {
        mip::Nanoseconds m_synced = reference_new.getTimestamp();
        mip::Nanoseconds m_old = reference_old.getTimestamp();
        if (m_synced < m_old)
        {
            throw std::invalid_argument("New reference timestamp < old reference timestamp.");
        }
        
        m_timestamp += (m_synced - m_old);
    }

    void Timestamp::setTimestamp(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }

    void Timestamp::setTimestamp(const Timestamp &from)
    {
        m_timestamp = convert(from.getTimestamp(), m_standard, from.m_standard);
    }

    void Timestamp::setWeek(Weeks week)
    {
        if (week < Weeks(0))         
        {
            throw std::invalid_argument("Week < 0.");
        }

        m_timestamp = week + getTimeOfWeek();
    }

    void Timestamp::setTimeOfWeek(Nanoseconds time)
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

    Nanoseconds Timestamp::getTimestamp() const
    {
        return m_timestamp;
    }

    Nanoseconds Timestamp::getTimeOfWeek()
    {
        return m_timestamp % Weeks(1);
    }
} // namespace mip
