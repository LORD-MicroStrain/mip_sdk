#pragma once

#include <microstrain/logging.h>
#include <mip/mip_packet.h>
#include <mip/mip_field.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif


// These functions are defined in cpp/mip/extras/pretty_print/debug_print.cpp.
// To use them, you must at least link the mip_extras library.
// If metadata support is enabled, you must also link the metadata library.
void mip_log_pretty_print_packet(const mip_packet_view* packet, microstrain_log_level level);
void mip_log_pretty_print_field(const mip_field_view* field, microstrain_log_level level);


////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{
////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_metadata_c  Mip Metadata [C]
///
///@brief %C functions which process information about the mip protocol.
///
/// To use these functions, the MIP SDK must have been compiled with metadata
/// support enabled (MIP_ENABLE_METADATA).
///
/// You must also link against the MIP pretty printing library, mip_pretty_print.
///
/// If referencing these functions causes a linker error, it is because you
/// haven't linked the pretty printing library or your copy of the MIP SDK has
/// not been compiled with metadata support.
///
/// These functions are implemented in the following files:
/// @li src/cpp/mip/extras/pretty_print/debug_print.cpp
/// @li src/cpp/mip/metadata/mip_all_definitions_runtime.cpp
///
///@{

struct mip_meta_descriptor_set_info;
struct mip_meta_field_info;
struct mip_meta_struct_info;
struct mip_meta_union_info;
struct mip_meta_param_info;

typedef int (*mip_meta_descriptor_set_callback)(void* userdata, const struct mip_meta_descriptor_set_info*);
typedef int (*mip_meta_field_callback         )(void* userdata, const struct mip_meta_field_info*         );
typedef int (*mip_meta_param_callback         )(void* userdata, const struct mip_meta_param_info*         );

int mip_meta_iterate_descriptor_sets(void* user, mip_meta_descriptor_set_callback callback);
const struct mip_meta_descriptor_set_info* mip_meta_find_descriptor_set(uint8_t descriptor);

uint8_t     mip_meta_descset_value(const struct mip_meta_descriptor_set_info* ds);
const char* mip_meta_descset_name(const struct mip_meta_descriptor_set_info* ds);

int mip_meta_iterate_fields(void* user, const struct mip_meta_descriptor_set_info* ds_info, mip_meta_field_callback callback);
const struct mip_meta_field_info* mip_meta_find_field_in_descriptor_set(const struct mip_meta_descriptor_set_info* ds_info, uint8_t field_descriptor, bool check_shared);

uint16_t mip_meta_field_descriptor(const struct mip_meta_field_info* info);
const struct mip_meta_field_info* mip_meta_field_response(const struct mip_meta_field_info* info);
uint8_t mip_meta_field_supported_function_selectors(const struct mip_meta_field_info* info);

const char* mip_meta_struct_name(const struct mip_meta_struct_info* info);
const char* mip_meta_struct_title(const struct mip_meta_struct_info* info);
const char* mip_meta_struct_docs(const struct mip_meta_struct_info* info);

int mip_meta_iterate_struct_parameters(void* user, const struct mip_meta_struct_info* info, mip_meta_param_callback callback);

const char*  mip_meta_param_name(const struct mip_meta_param_info* info);
const char*  mip_meta_param_docs(const struct mip_meta_param_info* info);
unsigned int mip_meta_param_type(const struct mip_meta_param_info* info);
uint8_t      mip_meta_param_required_functions(const struct mip_meta_param_info* info);
int          mip_meta_param_count(const struct mip_meta_param_info* info);


///@}
///@}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif
