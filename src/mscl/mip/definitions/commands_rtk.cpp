
#include "commands_rtk.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;
namespace commands_rtk {

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

void insert(MipSerializer& serializer, const GetStatusFlags& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetStatusFlags& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetImei& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetImei& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetImsi& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetImsi& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetIccid& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetIccid& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const ConnectedDeviceType& self)
{
    insert(serializer, self.function);
    insert(serializer, self.devType);
}

void extract(MipSerializer& serializer, ConnectedDeviceType& self)
{
    extract(serializer, self.function);
    extract(serializer, self.devType);
}

void insert(MipSerializer& serializer, const GetActCode& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetActCode& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetModemFirmwareVersion& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetModemFirmwareVersion& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetRssi& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetRssi& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const ServiceStatus& self)
{
    insert(serializer, self.reserved1);
    insert(serializer, self.reserved2);
}

void extract(MipSerializer& serializer, ServiceStatus& self)
{
    extract(serializer, self.reserved1);
    extract(serializer, self.reserved2);
}

void insert(MipSerializer& serializer, const ProdEraseStorage& self)
{
    insert(serializer, self.media);
}

void extract(MipSerializer& serializer, ProdEraseStorage& self)
{
    extract(serializer, self.media);
}

void insert(MipSerializer& serializer, const LedControl& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.altColor[i]);
    insert(serializer, self.act);
    insert(serializer, self.period);
}

void extract(MipSerializer& serializer, LedControl& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.altColor[i]);
    extract(serializer, self.act);
    extract(serializer, self.period);
}

void insert(MipSerializer& serializer, const ModemHardReset& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ModemHardReset& self)
{
    (void)serializer;
    (void)self;
}


} // namespace commands_rtk
} // namespace mscl

