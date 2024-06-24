#pragma once

#include <microstrain/common/platform.h>

#if __cpp_inline_variables >= 201606L
#define INLINE_VAR inline
#else
#define INLINE_VAR
#endif


#if __cpp_if_constexpr >= 201606L
    #define IF_CONSTEXPR if constexpr
#else
    #define IF_CONSTEXPR if
#endif
