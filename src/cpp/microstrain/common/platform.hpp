#pragma once

#include <microstrain/common/platform.h>


#if __cpp_if_constexpr >= 201606L
    #define IF_CONSTEXPR if constexpr
#else
    #define IF_CONSTEXPR if
#endif
