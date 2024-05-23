
#include "data_shared.hpp"

#include "microstrain/common/buffer.hpp"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
;

namespace C {
struct mip_interface;
} // namespace C

namespace data_shared {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(::microstrain::Buffer& serializer, const EventSource& self)
{
    insert(serializer, self.trigger_id);
    
}
void extract(::microstrain::Buffer& serializer, EventSource& self)
{
    extract(serializer, self.trigger_id);
    
}

void insert(::microstrain::Buffer& serializer, const Ticks& self)
{
    insert(serializer, self.ticks);
    
}
void extract(::microstrain::Buffer& serializer, Ticks& self)
{
    extract(serializer, self.ticks);
    
}

void insert(::microstrain::Buffer& serializer, const DeltaTicks& self)
{
    insert(serializer, self.ticks);
    
}
void extract(::microstrain::Buffer& serializer, DeltaTicks& self)
{
    extract(serializer, self.ticks);
    
}

void insert(::microstrain::Buffer& serializer, const GpsTimestamp& self)
{
    insert(serializer, self.tow);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.valid_flags);
    
}
void extract(::microstrain::Buffer& serializer, GpsTimestamp& self)
{
    extract(serializer, self.tow);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.valid_flags);
    
}

void insert(::microstrain::Buffer& serializer, const DeltaTime& self)
{
    insert(serializer, self.seconds);
    
}
void extract(::microstrain::Buffer& serializer, DeltaTime& self)
{
    extract(serializer, self.seconds);
    
}

void insert(::microstrain::Buffer& serializer, const ReferenceTimestamp& self)
{
    insert(serializer, self.nanoseconds);
    
}
void extract(::microstrain::Buffer& serializer, ReferenceTimestamp& self)
{
    extract(serializer, self.nanoseconds);
    
}

void insert(::microstrain::Buffer& serializer, const ReferenceTimeDelta& self)
{
    insert(serializer, self.dt_nanos);
    
}
void extract(::microstrain::Buffer& serializer, ReferenceTimeDelta& self)
{
    extract(serializer, self.dt_nanos);
    
}

void insert(::microstrain::Buffer& serializer, const ExternalTimestamp& self)
{
    insert(serializer, self.nanoseconds);
    
    insert(serializer, self.valid_flags);
    
}
void extract(::microstrain::Buffer& serializer, ExternalTimestamp& self)
{
    extract(serializer, self.nanoseconds);
    
    extract(serializer, self.valid_flags);
    
}

void insert(::microstrain::Buffer& serializer, const ExternalTimeDelta& self)
{
    insert(serializer, self.dt_nanos);
    
    insert(serializer, self.valid_flags);
    
}
void extract(::microstrain::Buffer& serializer, ExternalTimeDelta& self)
{
    extract(serializer, self.dt_nanos);
    
    extract(serializer, self.valid_flags);
    
}


} // namespace data_shared
} // namespace mip

