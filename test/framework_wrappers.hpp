#pragma once

#ifdef USE_DOCTEST
    #include <doctest/doctest.h>

    #define FAIL_AT_END_OF_TEST_IF_NOT_TRUE(condition) CHECK(condition)
    #define FAIL_IMMEDIATELY_IF_NOT_TRUE   (condition) REQUIRE(condition)
#endif