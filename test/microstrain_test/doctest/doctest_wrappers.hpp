#pragma once

#include <string>

#include <doctest/doctest.h>


#define CSTR_EQ_IMPL(actual, expected, level)                                                                 \
    do                                                                                                        \
    {                                                                                                         \
        INFO("Actual:   " << std::string(actual));                                                            \
        INFO("Expected: " << std::string(expected));                                                          \
                                                                                                              \
        bool terminated = actual[strlen(expected)] == '\0';                                                   \
        if (!terminated)                                                                                      \
        {                                                                                                     \
            level##_MESSAGE(terminated, "The actual string is not terminated when the expected string is.");  \
            break;                                                                                            \
        }                                                                                                     \
                                                                                                              \
        level##_EQ(strcmp(actual, expected), 0);                                                              \
    } while(0)
