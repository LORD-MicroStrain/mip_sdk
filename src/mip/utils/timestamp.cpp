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
    
    void TimestampExperimental::validateInputTime(const Nanoseconds &time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("time < 0");
        }
    }
} // namespace mip