
#include "data_system.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_system {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void BuiltInTest::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 16; i++)
        serializer.insert(result[i]);
    
}
void BuiltInTest::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 16; i++)
        serializer.extract(result[i]);
    
}

void TimeSyncStatus::insert(Serializer& serializer) const
{
    serializer.insert(time_sync);
    
    serializer.insert(last_pps_rcvd);
    
}
void TimeSyncStatus::extract(Serializer& serializer)
{
    serializer.extract(time_sync);
    
    serializer.extract(last_pps_rcvd);
    
}

void GpioState::insert(Serializer& serializer) const
{
    serializer.insert(states);
    
}
void GpioState::extract(Serializer& serializer)
{
    serializer.extract(states);
    
}

void GpioAnalogValue::insert(Serializer& serializer) const
{
    serializer.insert(gpio_id);
    
    serializer.insert(value);
    
}
void GpioAnalogValue::extract(Serializer& serializer)
{
    serializer.extract(gpio_id);
    
    serializer.extract(value);
    
}


} // namespace data_system
} // namespace mip

