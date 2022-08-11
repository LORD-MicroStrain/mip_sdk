
#include "data_system.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class MipSerializer;

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

void insert(MipSerializer& serializer, const BuiltInTest& self)
{
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.result[i]);
}

void extract(MipSerializer& serializer, BuiltInTest& self)
{
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.result[i]);
}

void insert(MipSerializer& serializer, const TimeSyncStatus& self)
{
    insert(serializer, self.time_sync);
    insert(serializer, self.last_pps_rcvd);
}

void extract(MipSerializer& serializer, TimeSyncStatus& self)
{
    extract(serializer, self.time_sync);
    extract(serializer, self.last_pps_rcvd);
}

void insert(MipSerializer& serializer, const GpioState& self)
{
    insert(serializer, self.states);
}

void extract(MipSerializer& serializer, GpioState& self)
{
    extract(serializer, self.states);
}

void insert(MipSerializer& serializer, const GpioAnalogValue& self)
{
    insert(serializer, self.gpio_id);
    insert(serializer, self.value);
}

void extract(MipSerializer& serializer, GpioAnalogValue& self)
{
    extract(serializer, self.gpio_id);
    extract(serializer, self.value);
}


} // namespace data_system
} // namespace mip

