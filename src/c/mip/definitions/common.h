#pragma once

////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{
///
///@defgroup MipCommands_c Mip Commands
///@brief Contains all MIP command definitions.
///
///@defgroup MipData_c Mip Data
///@brief Contains all MIP data definitions.
///
///@}
////////////////////////////////////////////////////////////////////////////////

#include <microstrain/serialization.h>
#include "../mip_field.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
#include <tuple>
#include <type_traits>
#include <utility>

namespace mip {
namespace C {
extern "C" {
using microstrain::C::microstrain_serializer;
#endif // __cplusplus

typedef struct mip_descriptor_rate
{
    uint8_t  descriptor;
    uint16_t decimation;
} mip_descriptor_rate;

void insert_mip_descriptor_rate(microstrain_serializer *serializer, const mip_descriptor_rate *self);
void extract_mip_descriptor_rate(microstrain_serializer *serializer, mip_descriptor_rate *self);

#define DECLARE_MIP_VECTOR_TYPE(n, type, name) \
typedef type name[n]; \
\
void insert_##name(microstrain_serializer* serializer, const name self); \
void extract_##name(microstrain_serializer* serializer, name self);

DECLARE_MIP_VECTOR_TYPE(3, float, mip_vector3f)
DECLARE_MIP_VECTOR_TYPE(4, float, mip_vector4f)
DECLARE_MIP_VECTOR_TYPE(9, float, mip_matrix3f)
DECLARE_MIP_VECTOR_TYPE(3, double, mip_vector3d)
DECLARE_MIP_VECTOR_TYPE(4, double, mip_vector4d)
DECLARE_MIP_VECTOR_TYPE(9, double, mip_matrix3d)
DECLARE_MIP_VECTOR_TYPE(4, float, mip_quatf)

#undef DECLARE_MIP_VECTOR_TYPE

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus
