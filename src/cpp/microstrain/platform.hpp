#pragma once

#include <microstrain/platform.h>

#ifdef __has_include
#if __has_include(<version>)
#include <version>
#endif
#endif

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

#if __cpp_lib_optional >= 201606L
#define MICROSTRAIN_HAS_OPTIONAL
#endif
