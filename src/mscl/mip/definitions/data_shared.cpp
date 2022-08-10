
#include "data_shared.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_shared {

using ::mscl::insert;
using ::mscl::extract;
using namespace ::mscl::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const EventSource& self)
{
    insert(serializer, self.trigger_id);
}

void extract(MipSerializer& serializer, EventSource& self)
{
    extract(serializer, self.trigger_id);
}

void insert(MipSerializer& serializer, const Ticks& self)
{
    insert(serializer, self.ticks);
}

void extract(MipSerializer& serializer, Ticks& self)
{
    extract(serializer, self.ticks);
}

void insert(MipSerializer& serializer, const DeltaTicks& self)
{
    insert(serializer, self.ticks);
}

void extract(MipSerializer& serializer, DeltaTicks& self)
{
    extract(serializer, self.ticks);
}

void insert(MipSerializer& serializer, const GpsTimestamp& self)
{
    insert(serializer, self.tow);
    insert(serializer, self.week_number);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GpsTimestamp& self)
{
    extract(serializer, self.tow);
    extract(serializer, self.week_number);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const DeltaTime& self)
{
    insert(serializer, self.seconds);
}

void extract(MipSerializer& serializer, DeltaTime& self)
{
    extract(serializer, self.seconds);
}

void insert(MipSerializer& serializer, const ReferenceTimestamp& self)
{
    insert(serializer, self.nanoseconds);
}

void extract(MipSerializer& serializer, ReferenceTimestamp& self)
{
    extract(serializer, self.nanoseconds);
}

void insert(MipSerializer& serializer, const ReferenceTimeDelta& self)
{
    insert(serializer, self.dt_nanos);
}

void extract(MipSerializer& serializer, ReferenceTimeDelta& self)
{
    extract(serializer, self.dt_nanos);
}

void insert(MipSerializer& serializer, const ExternalTimestamp& self)
{
    insert(serializer, self.nanoseconds);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, ExternalTimestamp& self)
{
    extract(serializer, self.nanoseconds);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const ExternalTimeDelta& self)
{
    insert(serializer, self.dt_nanos);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, ExternalTimeDelta& self)
{
    extract(serializer, self.dt_nanos);
    extract(serializer, self.valid_flags);
}


} // namespace data_shared
} // namespace mscl

