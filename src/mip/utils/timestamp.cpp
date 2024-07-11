#include "mip/utils/timestamp.hpp"

#include <stdexcept>

namespace mip
{
    TimestampExperimental::TimestampExperimental(const TimeStandard &standard) :
        m_standard(standard)
    {
        synchronize();
    }

    void TimestampExperimental::synchronize()
    {
        m_timestamp = m_standard.now();
    }

    Nanoseconds TimestampExperimental::getTimestamp() const
    {
        return m_timestamp;
    }
    
    void TimestampExperimental::setTimestamp(Nanoseconds time)
    {
        validateInputTime(time); 
        m_timestamp = time;
    }

    void TimestampExperimental::setWeek(Weeks week)
    {
        validateInputWeek(week);
        m_timestamp = week + getTimeOfWeek();
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
            throw std::invalid_argument("Time of week < one week.");
        }
        if (time > std::chrono::duration_cast<Nanoseconds>(Weeks(1)))
        {
            throw std::invalid_argument("Time of week > one week.");
        }
    }
    
    void TimestampExperimental::validateInputWeek(const Weeks &week)
    {
        if (week < Weeks(0))         
        {
            throw std::invalid_argument("Week < weeks in a year.");
        }
        if (week > std::chrono::duration_cast<Weeks>(Years(1)))
        {
            throw std::invalid_argument("Week > weeks in a year.");
        }
    }
} // namespace mip