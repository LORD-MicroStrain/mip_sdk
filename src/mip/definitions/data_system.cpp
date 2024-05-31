
#include "data_system.hpp"

#include "microstrain/common/serialization.hpp"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
;

namespace C {
struct mip_interface;
} // namespace C

namespace data_system {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(::microstrain::Serializer& serializer, const BuiltInTest& self)
{
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.result[i]);
    
}
void extract(::microstrain::Serializer& serializer, BuiltInTest& self)
{
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.result[i]);
    
}

void insert(::microstrain::Serializer& serializer, const TimeSyncStatus& self)
{
    insert(serializer, self.time_sync);
    
    insert(serializer, self.last_pps_rcvd);
    
}
void extract(::microstrain::Serializer& serializer, TimeSyncStatus& self)
{
    extract(serializer, self.time_sync);
    
    extract(serializer, self.last_pps_rcvd);
    
}

void insert(::microstrain::Serializer& serializer, const GpioState& self)
{
    insert(serializer, self.states);
    
}
void extract(::microstrain::Serializer& serializer, GpioState& self)
{
    extract(serializer, self.states);
    
}

void insert(::microstrain::Serializer& serializer, const GpioAnalogValue& self)
{
    insert(serializer, self.gpio_id);
    
    insert(serializer, self.value);
    
}
void extract(::microstrain::Serializer& serializer, GpioAnalogValue& self)
{
    extract(serializer, self.gpio_id);
    
    extract(serializer, self.value);
    
}


} // namespace data_system
} // namespace mip

