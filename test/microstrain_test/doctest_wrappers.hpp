#pragma once

#include <cstddef>

#include <doctest/doctest.h>

#define FRAMEWORK_WARN_TRUE(condition) WARN(condition)
#define FRAMEWORK_FAIL_TRUE(condition) CHECK(condition)
#define FRAMEWORK_EXIT_TRUE(condition) REQUIRE(condition)

namespace detail
{
}
