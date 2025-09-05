#pragma once

#ifdef USE_DOCTEST
    #include <doctest/doctest.h>

    #define WARN_IF_NOT_TRUE(condition) WARN(condition)
    #define EXPECT_TRUE(condition)      CHECK(condition)
    #define ASSERT_TRUE(condition)      REQUIRE(condition)

    #define WARN_IF_NOT_EQUAL(expected, actual) WARN_EQ(expected, actual)
    #define EXPECT_EQUAL(expected, actual) CHECK_EQ(expected, actual)
    #define ASSERT_EQUAL(expected, actual) REQUIRE_EQ(expected, actual)
#endif