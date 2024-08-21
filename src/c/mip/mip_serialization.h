#pragma once

#include "mip_field.h"

#include <microstrain/common/serialization.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif

void microstrain_serializer_init_new_field(microstrain_serializer* serializer, mip_packet_view* packet, uint8_t field_descriptor);
void microstrain_serializer_finish_new_field(const microstrain_serializer* serializer, mip_packet_view* packet);
void microstrain_serializer_init_from_field(microstrain_serializer* serializer, const mip_field_view* field);

#ifdef __cplusplus
} // namespace mip
} // namespace C
} // extern "C"
#endif
