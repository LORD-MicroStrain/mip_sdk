#include "mip/utils/timestamp.hpp"

#include <stdexcept>

namespace mip
{
    TimestampExperimental::TimestampExperimental(const TimeStandard &standard) :
        m_standard(standard)
    {
        m_timestamp = mip::Nanoseconds(0);
    }

    Nanoseconds TimestampExperimental::getTimestamp()
    {
        return m_timestamp;
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
    
    void TimestampExperimental::setTimestamp(Nanoseconds time)
    {
        validateInputTime(time); 
        m_timestamp = time;
    }
    
    void TimestampExperimental::validateInputTime(const Nanoseconds &time)
    {
        if (time < Nanoseconds(0))
        {
            throw std::invalid_argument("time < 0");
        }
    }
} // namespace mip