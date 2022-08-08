
#include "commands_system.h"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_system_comm_mode_command(struct mip_serializer* serializer, const struct mip_system_comm_mode_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    insert_u8(serializer, self->mode);
}

void extract_mip_system_comm_mode_command(struct mip_serializer* serializer, struct mip_system_comm_mode_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    extract_u8(serializer, &self->mode);
}


#ifdef __cplusplus
} // namespace C
} // namespace mscl
} // extern "C"
#endif // __cplusplus

