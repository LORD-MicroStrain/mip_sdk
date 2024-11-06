#include "mip/extras/time/timestamp.hpp"

#include "mip/extras/time/misc.hpp"

namespace mip
{
    TimestampManager::TimestampManager(const mip::TimeStandard &standard) :
        m_standard(standard)
    {
        synchronize();
    }

    void TimestampManager::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    void TimestampManager::increment(const TimestampManager &reference_new, const TimestampManager &reference_old)
    {
        mip::Nanoseconds m_synced = reference_new.getTimestamp();
        mip::Nanoseconds m_old = reference_old.getTimestamp();
        if (m_synced < m_old)
        {
            throw std::invalid_argument("New reference timestamp < old reference timestamp.");
        }

        m_timestamp += (m_synced - m_old);
    }

    void TimestampManager::setTimestamp(Nanoseconds time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0.");
        }

        m_timestamp = time;
    }

    void TimestampManager::setTimestamp(const TimestampManager &from)
    {
        m_timestamp = convert(from.getTimestamp(), m_standard, from.m_standard);
    }

    void TimestampManager::setWeek(Weeks week)
    {
        if (week < Weeks(0))
        {
            throw std::invalid_argument("Week < 0.");
        }

        m_timestamp = week + getTimeOfWeek();
    }

    void TimestampManager::setTimeOfWeek(Nanoseconds time)
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

    Nanoseconds TimestampManager::getTimestamp() const
    {
        return m_timestamp;
    }

    Nanoseconds TimestampManager::getTimeOfWeek() const
    {
        return m_timestamp % Weeks(1);
    }
} // namespace mip
