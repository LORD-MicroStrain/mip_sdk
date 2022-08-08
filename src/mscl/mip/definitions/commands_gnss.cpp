
#include "commands_gnss.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;
namespace commands_gnss {

namespace C {
struct mip_interface;
} // namespace C

using ::mscl::insert;
using ::mscl::extract;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ReceiverInfo& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const SignalConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.gps_enable);
    insert(serializer, self.glonass_enable);
    insert(serializer, self.galileo_enable);
    insert(serializer, self.beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.reserved[i]);
}

void extract(MipSerializer& serializer, SignalConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.gps_enable);
    extract(serializer, self.glonass_enable);
    extract(serializer, self.galileo_enable);
    extract(serializer, self.beidou_enable);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.reserved[i]);
}

void insert(MipSerializer& serializer, const RtkDongleConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reserved[i]);
}

void extract(MipSerializer& serializer, RtkDongleConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reserved[i]);
}

void insert(MipSerializer& serializer, const ReceiverSafeMode& self)
{
    insert(serializer, self.receiver_id);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, ReceiverSafeMode& self)
{
    extract(serializer, self.receiver_id);
    extract(serializer, self.enable);
}


} // namespace commands_gnss
} // namespace mscl

