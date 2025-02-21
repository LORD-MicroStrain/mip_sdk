
#include "common.h"

#include "microstrain/serialization.h"


void insert_mip_descriptor_rate(microstrain_serializer* serializer, const mip_descriptor_rate* self)
{
    microstrain_insert_u8(serializer, self->descriptor);
    microstrain_insert_u16(serializer, self->decimation);
}

void extract_mip_descriptor_rate(microstrain_serializer* serializer, mip_descriptor_rate* self)
{
    microstrain_extract_u8(serializer, &self->descriptor);
    microstrain_extract_u16(serializer, &self->decimation);
}

#define IMPLEMENT_MIP_VECTOR_FUNCTIONS(n,type,name) \
void insert_##name(microstrain_serializer* serializer, const name self) \
{ \
    for(unsigned int i=0; i<n; i++) \
        microstrain_insert_##type(serializer, self[i]); \
} \
void extract_##name(microstrain_serializer* serializer, name self) \
{ \
    for(unsigned int i=0; i<n; i++) \
        microstrain_extract_##type(serializer, &self[i]); \
}

IMPLEMENT_MIP_VECTOR_FUNCTIONS(3, float,  mip_vector3f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4, float,  mip_vector4f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(9, float,  mip_matrix3f)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(3, double, mip_vector3d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4, double, mip_vector4d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(9, double, mip_matrix3d)
IMPLEMENT_MIP_VECTOR_FUNCTIONS(4, float,  mip_quatf)

#undef IMPLEMENT_MIP_VECTOR_FUNCTIONS
