
#include "commands_system.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;
namespace commands_system {

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

void insert(MipSerializer& serializer, const CommMode& self)
{
    insert(serializer, self.function);
    insert(serializer, self.mode);
}

void extract(MipSerializer& serializer, CommMode& self)
{
    extract(serializer, self.function);
    extract(serializer, self.mode);
}


} // namespace commands_system
} // namespace mscl

