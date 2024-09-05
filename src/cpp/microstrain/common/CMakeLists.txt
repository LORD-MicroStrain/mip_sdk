# microstrain_common is defined by the C api.
target_sources(${MICROSTRAIN_COMMON_LIBRARY} PRIVATE
    embedded_time.hpp
    index.hpp
    logging.hpp
    platform.hpp
    serialization.hpp
    serialization/readwrite.hpp
    serialization/serializer.hpp
)

target_compile_features(${MICROSTRAIN_COMMON_LIBRARY} PUBLIC cxx_std_11)
target_include_directories(${MICROSTRAIN_COMMON_LIBRARY} INTERFACE ${SRC_CPP_DIR})

#
# Installation
#
include(microstrain_utilities)
microstrain_setup_install_headers(${MICROSTRAIN_COMMON_LIBRARY})
