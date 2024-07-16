
#include "data_shared.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_shared {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void EventSource::insert(Serializer& serializer) const
{
    serializer.insert(trigger_id);
    
}
void EventSource::extract(Serializer& serializer)
{
    serializer.extract(trigger_id);
    
}

void Ticks::insert(Serializer& serializer) const
{
    serializer.insert(ticks);
    
}
void Ticks::extract(Serializer& serializer)
{
    serializer.extract(ticks);
    
}

void DeltaTicks::insert(Serializer& serializer) const
{
    serializer.insert(ticks);
    
}
void DeltaTicks::extract(Serializer& serializer)
{
    serializer.extract(ticks);
    
}

void GpsTimestamp::insert(Serializer& serializer) const
{
    serializer.insert(tow);
    
    serializer.insert(week_number);
    
    serializer.insert(valid_flags);
    
}
void GpsTimestamp::extract(Serializer& serializer)
{
    serializer.extract(tow);
    
    serializer.extract(week_number);
    
    serializer.extract(valid_flags);
    
}

void DeltaTime::insert(Serializer& serializer) const
{
    serializer.insert(seconds);
    
}
void DeltaTime::extract(Serializer& serializer)
{
    serializer.extract(seconds);
    
}

void ReferenceTimestamp::insert(Serializer& serializer) const
{
    serializer.insert(nanoseconds);
    
}
void ReferenceTimestamp::extract(Serializer& serializer)
{
    serializer.extract(nanoseconds);
    
}

void ReferenceTimeDelta::insert(Serializer& serializer) const
{
    serializer.insert(dt_nanos);
    
}
void ReferenceTimeDelta::extract(Serializer& serializer)
{
    serializer.extract(dt_nanos);
    
}

void ExternalTimestamp::insert(Serializer& serializer) const
{
    serializer.insert(nanoseconds);
    
    serializer.insert(valid_flags);
    
}
void ExternalTimestamp::extract(Serializer& serializer)
{
    serializer.extract(nanoseconds);
    
    serializer.extract(valid_flags);
    
}

void ExternalTimeDelta::insert(Serializer& serializer) const
{
    serializer.insert(dt_nanos);
    
    serializer.insert(valid_flags);
    
}
void ExternalTimeDelta::extract(Serializer& serializer)
{
    serializer.extract(dt_nanos);
    
    serializer.extract(valid_flags);
    
}


} // namespace data_shared
} // namespace mip

