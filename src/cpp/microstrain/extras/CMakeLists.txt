set(MICROSTRAIN_EXTRAS_LIBRARY "microstrain_extras" CACHE INTERNAL "Name of the MicroStrain extras library")
mark_as_advanced(MICROSTRAIN_EXTRAS_LIBRARY)

set(SOURCES
    scope_helper.cpp
    scope_helper.hpp
)

add_library(${MICROSTRAIN_EXTRAS_LIBRARY} ${SOURCES})

target_compile_features(${MICROSTRAIN_EXTRAS_LIBRARY} PUBLIC cxx_std_11)
target_link_libraries(${MICROSTRAIN_EXTRAS_LIBRARY}
    PRIVATE ${MICROSTRAIN_PRIVATE_LIBRARY}
    PUBLIC ${MICROSTRAIN_COMMON_LIBRARY}
)
target_compile_definitions(${MICROSTRAIN_EXTRAS_LIBRARY} PUBLIC "MICROSTRAIN_ENABLE_EXTRAS=1")

target_link_libraries(${MICROSTRAIN_LIBRARIES} INTERFACE ${MICROSTRAIN_EXTRAS_LIBRARY})
