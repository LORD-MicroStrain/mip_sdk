#pragma once

#include "mip_field.h"

#include <microstrain/serialization.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

typedef microstrain_serializer mip_serializer;

void mip_serializer_init_new_field(mip_serializer* serializer, mip_packet_view* packet, uint8_t field_descriptor);
void mip_serializer_finish_new_field(const mip_serializer* serializer, mip_packet_view* packet);
void microstrain_serializer_init_from_field(mip_serializer* serializer, const mip_field_view* field);

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif
