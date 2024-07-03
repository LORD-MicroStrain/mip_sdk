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
} // namespace mip