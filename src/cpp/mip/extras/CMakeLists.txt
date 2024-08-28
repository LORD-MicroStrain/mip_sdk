set(MIP_EXTRAS_LIBRARY "mip_extras" CACHE INTERNAL "Name of the MIP extras library")
mark_as_advanced(MIP_EXTRAS_LIBRARY)

add_library(${MIP_EXTRAS_LIBRARY}
    composite_result.hpp
    descriptor_id.hpp
    firmware_version.cpp
    firmware_version.hpp
)

target_compile_features(${MIP_EXTRAS_LIBRARY} PUBLIC cxx_std_11)
target_link_libraries(${MIP_EXTRAS_LIBRARY}
    PRIVATE ${MICROSTRAIN_PRIVATE_LIBRARY}
    PUBLIC ${MIP_LIBRARY} ${MICROSTRAIN_EXTRAS_LIBRARY}
)
target_compile_definitions(${MIP_EXTRAS_LIBRARY} PUBLIC "MIP_ENABLE_EXTRAS=1")

target_link_libraries(${MIP_LIBRARIES} INTERFACE ${MIP_EXTRAS_LIBRARY})
