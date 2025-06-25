#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus

#include <microstrain/embedded_time.hpp>

namespace mip {
namespace C {
extern "C" {

typedef microstrain::C::microstrain_embedded_timestamp mip_timestamp;
typedef microstrain::C::microstrain_embedded_timestamp mip_timeout;

#else

#include <microstrain/embedded_time.h>

typedef microstrain_embedded_timestamp mip_timestamp;
typedef microstrain_embedded_timestamp mip_timeout;

#endif


#ifdef MIP_ENABLE_DIAGNOSTICS

// Saturating addition
#define MIP_DIAG_INC(counter, amount) do { if (counter + amount < counter) counter = -1; else counter += amount; } while(false)

#define MIP_DIAG_ZERO(counter) counter = 0

#else // MIP_ENABLE_DIAGNOSTICS

// Do nothing if diagnostic counters diabled. Cast amount to void to avoid "unused local variable" warnings.
#define MIP_DIAG_INC(counter, amount) (void)amount

#define MIP_DIAG_ZERO(counter) (void)0

#endif // MIP_ENABLE_DIAGNOSTICS


#ifdef __cplusplus

} // extern "C"
} // namespace C

using Timestamp = microstrain::EmbeddedTimestamp;
using Timeout   = microstrain::EmbeddedTimeout;

} // namespace mip

#endif
