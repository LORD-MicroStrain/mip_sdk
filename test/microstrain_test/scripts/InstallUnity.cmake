include(FetchContent)

FetchContent_Declare(unity
    GIT_REPOSITORY "https://github.com/ThrowTheSwitch/Unity.git"
    GIT_TAG "v2.6.1"
)

FetchContent_MakeAvailable(unity)

# Unity doesn't have native CMake support, so we have to build it manually
if (NOT TARGET unity)
    add_library(unity STATIC ${unity_SOURCE_DIR}/src/unity.c)
    target_include_directories(unity PUBLIC ${unity_SOURCE_DIR}/src)
endif()

# Disable Spectre mitigation warnings for Unity as building with /Qspectre would add performance
# overhead without a significant security threat to justify it.
if(MSVC)
    target_compile_options(unity PRIVATE /wd5045)
endif()
