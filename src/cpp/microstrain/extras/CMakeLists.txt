
set(SOURCES
    scope_helper.cpp
    scope_helper.hpp
)

add_library(microstrain_extras ${SOURCES})
target_compile_features(microstrain_extras PUBLIC cxx_std_11)
target_include_directories(microstrain_extras INTERFACE ${MICROSTRAIN_INCLUDE_DIR})

