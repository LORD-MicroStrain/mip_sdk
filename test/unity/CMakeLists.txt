include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY "https://github.com/ThrowTheSwitch/Unity.git"
    GIT_TAG "v2.6.1"
    FIND_PACKAGE_ARGS NAMES unity
)

FetchContent_MakeAvailable(Unity)

if (NOT TARGET Unity)
    add_library(Unity STATIC ${unity_SOURCE_DIR}/src/unity.c)
    target_include_directories(Unity PUBLIC ${unity_SOURCE_DIR}/src)
endif()
