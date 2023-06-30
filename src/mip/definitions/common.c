
#include "common.h"

#include "../utils/serialization.h"

#ifdef __cplusplus
namespace mip {
extern "C" {
#endif // __cplusplus


void insert_mip_descriptor_rate(mip_serializer* serializer, const mip_descriptor_rate* self)
{
    insert_u8(serializer, self->descriptor);
    insert_u16(serializer, self->decimation);
}

void extract_mip_descriptor_rate(mip_serializer* serializer, mip_descriptor_rate* self)
{
    extract_u8(serializer, &self->descriptor);
    extract_u16(serializer, &self->decimation);
}

#define IMPLEMENT_MIP_VECTOR_FUNCTIONS(n,type,name) \
void insert_mip_##name(mip_serializer* serializer, const mip_##name* self) \
{ \
    for(unsigned int i=0; i<n; i++) \
        insert_##type(serializer, self->v[i]); \
} \
void extract_mip_##name(mip_serializer* serializer, mip_##name* self) \
{ \
    for(unsigned int i=0; i<n; i++) \
        extract_##type(serializer, &self->v[i]); \
}

IMPLEMENT_MIP_VECTOR_FUNCTIONS(3,float,vector3f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4,float,vector4f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(9,float,matrix3f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(3,double,vector3d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4,double,vector4d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(9,double,matrix3d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4,float,quatf)

#undef IMPLEMENT_MIP_VECTOR_FUNCTIONS

#ifdef __cplusplus
} // namespace mip
} // extern "C"
#endif // __cplusplus
