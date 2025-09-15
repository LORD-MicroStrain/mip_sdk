set(MICROSTRAIN_RECORDING_LIBRARY "microstrain_recording" CACHE INTERNAL "Name of the MicroStrain recording connections library")
mark_as_advanced(MICROSTRAIN_RECORDING_LIBRARY)

set(MICROSTRAIN_RECORDING_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/recording_connection.c"
    "${CMAKE_CURRENT_LIST_DIR}/recording_connection.h"
)

add_library(${MICROSTRAIN_RECORDING_LIBRARY}
    "${MICROSTRAIN_RECORDING_SOURCES}"
)

# Configure the project in a sub-folder for IDEs that support it
file(RELATIVE_PATH PROJECT_RELATIVE_PATH "${MICROSTRAIN_SRC_DIR}" "${CMAKE_CURRENT_LIST_DIR}")
set_target_properties(${MICROSTRAIN_RECORDING_LIBRARY} PROPERTIES
    FOLDER "${PROJECT_RELATIVE_PATH}"
)

if(MSVC)
    source_group(TREE ${MICROSTRAIN_SRC_DIR} FILES ${MICROSTRAIN_RECORDING_SOURCES})

    # Disable warnings about using "insecure" functions since it's not cross platform compatible
    target_compile_definitions(${MICROSTRAIN_RECORDING_LIBRARY} PRIVATE "_CRT_SECURE_NO_WARNINGS")
endif()

target_compile_features(${MICROSTRAIN_RECORDING_LIBRARY} PUBLIC c_std_11)

target_link_libraries(${MICROSTRAIN_RECORDING_LIBRARY} PUBLIC ${MICROSTRAIN_LIBRARY})

target_compile_options(${MICROSTRAIN_RECORDING_LIBRARY} PRIVATE ${MICROSTRAIN_PRIVATE_COMPILE_OPTIONS})

set(MICROSTRAIN_LIBRARIES_TMP ${MICROSTRAIN_LIBRARIES_TMP} ${MICROSTRAIN_RECORDING_LIBRARY} PARENT_SCOPE)

#
# Installation
#

include(microstrain_utilities)
microstrain_setup_library_install(${MICROSTRAIN_RECORDING_LIBRARY} ${MICROSTRAIN_SRC_DIR})
