#include "mip/utils/timestamp.hpp"

#include <stdexcept>

namespace mip
{
    TimestampExperimental::TimestampExperimental(const TimeStandard &standard) :
        m_standard(standard)
    {
        m_timestamp = mip::Nanoseconds(0);
    }

    TimestampExperimental TimestampExperimental::Now(const TimeStandard &standard)
    {
        TimestampExperimental timestamp(standard);
        timestamp.synchronize();
        return timestamp;
    }
    
    void TimestampExperimental::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    Nanoseconds TimestampExperimental::getTimestamp()
    {
        return m_timestamp;
    }
    
    void TimestampExperimental::setTimestamp(Nanoseconds time)
    {
        validateInputTime(time); 
        m_timestamp = time;
    }

    Nanoseconds TimestampExperimental::getTimeOfWeek()
    {
        return m_timestamp % Weeks(1);
    }
    
    void TimestampExperimental::setTimeOfWeek(Nanoseconds time)
    {
        validateInputTimeOfWeek(time);
        m_timestamp = std::chrono::duration_cast<Weeks>(m_timestamp) + time;
    }
    
    void TimestampExperimental::validateInputTime(const Nanoseconds &time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time < 0");
        }
    }

    void TimestampExperimental::validateInputTimeOfWeek(const Nanoseconds &time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("Time of week < one week");
        }
        if (time > std::chrono::duration_cast<Nanoseconds>(Weeks(1)))
        {
            throw std::invalid_argument("Time of week > one week.");
        }
    }
} // namespace mip