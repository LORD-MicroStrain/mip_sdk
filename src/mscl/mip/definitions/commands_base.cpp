
#include "commands_base.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;
namespace commands_base {

namespace C {
struct mip_interface;
} // namespace C

using ::mscl::insert;
using ::mscl::extract;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const BaseDeviceInfo& self)
{
    insert(serializer, self.firmware_version);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        insert(serializer, self.device_options[i]);
}

void extract(MipSerializer& serializer, BaseDeviceInfo& self)
{
    extract(serializer, self.firmware_version);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.model_name[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.model_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.serial_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.lot_number[i]);
    for(unsigned int i=0; i < 16; i++)
        extract(serializer, self.device_options[i]);
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const Ping& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, Ping& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const SetIdle& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, SetIdle& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetDeviceInfo& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetDeviceInfo& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetDeviceDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetDeviceDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const BuiltInTest& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, BuiltInTest& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const Resume& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, Resume& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GetExtendedDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GetExtendedDescriptors& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const ContinuousBit& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ContinuousBit& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const CommSpeed& self)
{
    insert(serializer, self.function);
    insert(serializer, self.port);
    insert(serializer, self.baud);
}

void extract(MipSerializer& serializer, CommSpeed& self)
{
    extract(serializer, self.function);
    extract(serializer, self.port);
    extract(serializer, self.baud);
}

void insert(MipSerializer& serializer, const GpsTimeUpdate& self)
{
    insert(serializer, self.function);
    insert(serializer, self.field_id);
    insert(serializer, self.value);
}

void extract(MipSerializer& serializer, GpsTimeUpdate& self)
{
    extract(serializer, self.function);
    extract(serializer, self.field_id);
    extract(serializer, self.value);
}

void insert(MipSerializer& serializer, const SoftReset& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, SoftReset& self)
{
    (void)serializer;
    (void)self;
}


} // namespace commands_base
} // namespace mscl

