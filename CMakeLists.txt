cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 20) # CMake decays this to the highest supported by the compiler
set(CMAKE_C_STANDARD 11)

project(
    "MIP_SDK"
    VERSION "3.1.00"
    DESCRIPTION "MicroStrain Communications Library for embedded systems"
    LANGUAGES C CXX
)

set(MICROSTRAIN_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")

# Add our cmake directory to the available cmake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

#
# Build options
#

# MicroStrain options
option(MICROSTRAIN_ENABLE_LOGGING "Build with logging functions enabled" ON)

set(MICROSTRAIN_LOGGING_MAX_LEVEL "MICROSTRAIN_LOG_LEVEL_INFO" CACHE STRING "Max log level the SDK is allowed to log. If this is defined, any log level logged at a higher level than this will result in a noop regardless of runtime configuration.")
set(MICROSTRAIN_TIMESTAMP_TYPE "uint64_t" CACHE STRING "Override the type used for received data timestamps and timeouts (must be unsigned or at least 64 bits).")

include(check_cxx_support)

option(MICROSTRAIN_ENABLE_CPP "Enables the C++ API. Turn off to avoid compiling the C++ API." ON)
#option(MICROSTRAIN_ENABLE_CPP_C_NAMESPACE "Wraps the C api in a C++ namespace to avoid global namespace pollution (e.g. microstrain::C::microstrain_serial_port)" ${MICROSTRAIN_ENABLE_CPP})
message(DEBUG "MICROSTRAIN_ENABLE_CPP=${MICROSTRAIN_ENABLE_CPP}")
if(MICROSTRAIN_ENABLE_CPP)
    option(MICROSTRAIN_USE_STD_SPAN   "Use std::span from C++20 for microstrain::Span." ${MICROSTRAIN_COMPILER_SUPPORTS_SPAN})
    option(MICROSTRAIN_USE_STD_ENDIAN "Use std::endian from C++20 for microstrain::Endian." ${MICROSTRAIN_COMPILER_SUPPORTS_BIT})
    message(DEBUG "MICROSTRAIN_USE_STD_SPAN=${MICROSTRAIN_USE_STD_SPAN}")
    message(DEBUG "MICROSTRAIN_USE_STD_ENDIAN=${MICROSTRAIN_USE_STD_ENDIAN}")
endif()

option(MICROSTRAIN_ENABLE_EXTRAS "Build extra support into the library including some things that might work at a higher level and use dynamically allocated memory" ON)
option(MICROSTRAIN_ENABLE_SERIAL "Build serial connection support into the library and examples" ON)
option(MICROSTRAIN_ENABLE_TCP "Build TCP connection support into the library and examples" ON)

option(MICROSTRAIN_BUILD_PACKAGE "Whether to build a package from the resulting binaries" OFF)
option(MICROSTRAIN_BUILD_EXAMPLES "Builds the example programs." OFF)

# CTest defines this option to ON by default, so override it to OFF here.
option(MICROSTRAIN_BUILD_TESTS "Build the testing tree." OFF)

option(MICROSTRAIN_BUILD_DOCUMENTATION       "Build the documentation." OFF)
option(MICROSTRAIN_BUILD_DOCUMENTATION_FULL  "Build the full (internal) documentation." OFF)
option(MICROSTRAIN_BUILD_DOCUMENTATION_QUIET "Suppress doxygen standard output." ON)

option(MICROSTRAIN_CMAKE_DEBUG "If set, prints build system debug info such as source files and preprocessor definitions." ON)  # TODO: off by default
mark_as_advanced(MICROSTRAIN_CMAKE_DEBUG)

# MIP options
option(MIP_ENABLE_DIAGNOSTICS "Enable various diagnostic counters in the mip library for debugging." OFF)

if(NOT DEFINED MIP_ENABLE_METADATA)
    message(STATUS "MIP_ENABLE_METADATA not set - compiler support: ${MIP_COMPILER_SUPPORTS_METADATA}")
endif()
option(MIP_ENABLE_METADATA "Build support for MIP protocol metadata in C++ (requires c++20)" ${MIP_COMPILER_SUPPORTS_METADATA})
option(MIP_ENABLE_EXTRAS "Build extra support into the library including some things that might work at a higher level and use dynamically allocated memory" ${MICROSTRAIN_ENABLE_EXTRAS})

# Include some utilities used for MicroStrain projects
include(microstrain_utilities)

#
# Common preprocessor definitions
#

if(WIN32)
    if(MSVC)
        set(MICROSTRAIN_PRIVATE_COMPILE_OPTIONS
            "/external:anglebrackets" # Treat angle brackets as system includes
            "/external:W0"            # No warnings from system includes
            "/W4"                     # Enable warnings
            "/WX"                     # Warnings as errors
            "/Zc:__cplusplus"         # Enable updated __cplusplus value
            "/MP"                     # Multi-process compilation
        )

        if(${MSVC_TOOLSET_VERSION} LESS 143)
            list(APPEND MICROSTRAIN_PRIVATE_COMPILE_OPTIONS
                "/wd4201" # Suppress warnings related to nameless structs/unions
            )
        endif()
    endif()

    # Disable windows defined min/max
    # Set Windows header version (0x0501 is _WIN32_WINNT_WINXP, required for TCP)
    set(MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS
        "NOMINMAX=1"
        "_WIN32_WINNT=_WIN32_WINNT_WINXP"
    )
else()
    set(MICROSTRAIN_PRIVATE_COMPILE_OPTIONS
        "-Wall"   # Enable warnings
        "-Wextra" # Enable extra warnings
        "-Werror" # Warnings as errors
    )

    set(MICROSTRAIN_PRIVATE_COMPILE_DEFINITIONS "")
endif()

# Suppress warnings due to adding files with target_sources.
cmake_policy(SET CMP0076 NEW)

# Suppress warnings due to linking libraries across directories
cmake_policy(SET CMP0079 NEW)

#
# Version numbering
#

microstrain_get_git_version(MICROSTRAIN_GIT_VERSION MICROSTRAIN_GIT_VERSION_CLEAN)


# Add all the C files (required even for C++ API)
set(MICROSTRAIN_SRC_C_DIR "${MICROSTRAIN_SRC_DIR}/c")

add_subdirectory(${MICROSTRAIN_SRC_C_DIR}/microstrain)
add_subdirectory(${MICROSTRAIN_SRC_C_DIR}/mip)

# Add all the C++ files (if C++ enabled)
if(MICROSTRAIN_ENABLE_CPP)
    set(MICROSTRAIN_SRC_CPP_DIR "${MICROSTRAIN_SRC_DIR}/cpp")

    add_subdirectory(${MICROSTRAIN_SRC_CPP_DIR}/microstrain)
    add_subdirectory(${MICROSTRAIN_SRC_CPP_DIR}/mip)

    #target_compile_definitions(${MICROSTRAIN_LIBRARY} PUBLIC "MICROSTRAIN_ENABLE_CPP_C_NAMESPACE=${MICROSTRAIN_ENABLE_CPP_C_NAMESPACE}")
endif()

# Create list of requested libraries for easier linking
# Use the PARENT_SCOPE option when adding new modules and make sure it propagates up all levels
set(MICROSTRAIN_LIBRARIES ${MICROSTRAIN_LIBRARIES_TMP} CACHE STRING "List of all requested MicroStrain libraries to make linking easier" FORCE)
set(MIP_LIBRARIES ${MIP_LIBRARIES_TMP} CACHE STRING "List of all requested MIP libraries to make linking easier" FORCE)

#
# Libraries
#

#
# TESTING
#


if(MICROSTRAIN_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory("test")
endif()

#
# EXAMPLES
#

if(MICROSTRAIN_BUILD_EXAMPLES)
    add_subdirectory("examples")
endif()

#
# DOCUMENTATION
#

if(MICROSTRAIN_BUILD_DOCUMENTATION)
    find_package(Doxygen)

    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is required to build documentation.")
    endif()

    set(DOXYGEN_PROJECT_NUMBER "${MICROSTRAIN_GIT_VERSION}")

    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/documentation")
    set(DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_LIST_DIR}/docs")

    set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)

    if(MICROSTRAIN_ENABLE_CPP)
        set(DOXYGEN_PREDEFINED "__cplusplus")
    endif()

    set(DOXYGEN_EXTRACT_ALL YES)

    if(MICROSTRAIN_BUILD_DOCUMENTATION_FULL)
        set(DOXYGEN_INTERNAL_DOCS YES)
        # set(DOXYGEN_WARN_AS_ERROR YES)
    else()
        set(DOXYGEN_HIDE_UNDOC_MEMBERS YES)
        set(DOXYGEN_HIDE_UNDOC_CLASSES YES)
    endif()

    if(MICROSTRAIN_BUILD_DOCUMENTATION_QUIET)
        set(DOXYGEN_QUIET YES)
    endif()

    set(DOXYGEN_HTML_EXTRA_STYLESHEET "${CMAKE_CURRENT_LIST_DIR}/docs/style.css")

    doxygen_add_docs(docs
        "${MICROSTRAIN_SRC_DIR}" "${CMAKE_CURRENT_LIST_DIR}/docs"
        COMMENT "Generating documentation."
    )

    # Add a target to enable users to zip up the docs
    add_custom_target(package_docs
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/documentation/html
        DEPENDS docs
        COMMAND ${CMAKE_COMMAND} -E tar "cf" "${CMAKE_CURRENT_BINARY_DIR}/mipsdk_${MICROSTRAIN_GIT_VERSION}_Documentation.zip" --format=zip "."
    )
endif()

#
# Packaging
#

# If we were asked to package, find the generators we can use
if(MICROSTRAIN_BUILD_PACKAGE)
    microstrain_get_architecture(MICROSTRAIN_ARCH)

    set(MIP_FILE_NAME_PREFIX "mipsdk_${MICROSTRAIN_GIT_VERSION}_${MICROSTRAIN_ARCH}")

    set(FOUND_CPACK_GENERATORS "")

    set(DPKG_ROOT "" CACHE STRING "Location of the dpkg executable")
    find_program(DPKG_EXECUTABLE
        NAMES dpkg
        PATHS ${DPKG_ROOT}
        DOC "dpkg command line client"
    )

    if(NOT ${DPKG_EXECUTABLE} STREQUAL "DPKG_EXECUTABLE-NOTFOUND")
        list(APPEND FOUND_CPACK_GENERATORS "DEB")

        # DEB specific configuration
        # Build different deb packages for each target
        set(CPACK_DEBIAN_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}.deb")
        set(CPACK_DEB_COMPONENT_INSTALL ON)
        set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
    endif()

    set(RPMBUILD_ROOT "" CACHE STRING "Location of the rpmbuild executable")
    find_program(RPMBUILD_EXECUTABLE
        NAMES rpmbuild
        PATHS ${RPMBUILD_ROOT}
        DOC "rpmbuild command line client"
    )

    if(NOT ${RPMBUILD_EXECUTABLE} STREQUAL "RPMBUILD_EXECUTABLE-NOTFOUND")
        list(APPEND FOUND_CPACK_GENERATORS "RPM")

        # RPM specific configuration
        # Build different RPM packages for each target
        set(CPACK_RPM_MIP_FILE_NAME "${MIP_FILE_NAME_PREFIX}.rpm")
        set(CPACK_RPM_COMPONENT_INSTALL ON)
        set(CPACK_RPM_PACKAGE_AUTOREQ ON)
    endif()

    # Windows always has zip installed, so only look for it on linux and mac
    if(WIN32)
        list(APPEND FOUND_CPACK_GENERATORS "ZIP")
    else()
        set(ZIP_ROOT "" CACHE STRING "Location of the zip executable")
        find_program(ZIP_EXECUTABLE
            NAMES zip
            PATHS ${ZIP_ROOT}
            DOC "zip command line client"
        )
        if(NOT ${ZIP_EXECUTABLE} STREQUAL "ZIP_EXECUTABLE-NOTFOUND")
            list(APPEND FOUND_CPACK_GENERATORS "ZIP")
        endif()
    endif()

    if(NOT FOUND_CPACK_GENERATORS)
        message(FATAL_ERROR "Unable to find a suitable package generator, but we were requested to build a package.")
    else()
        message(STATUS "Packaging using the following generators: ${FOUND_CPACK_GENERATORS}")
    endif()

    # Packaging
    # NOTE: CPack requires all these variables to be set before importing the module. Do not move them after the include(CPack) line
    set(CPACK_GENERATOR "${FOUND_CPACK_GENERATORS}")
    set(CPACK_PACKAGE_VENDOR "MicroStrain by HBK")
    set(CPACK_PACKAGE_CONTACT "MicroStrain Support <microstrainsupport@hbkworld.com>")

    set(CPACK_PACKAGE_VERSION ${MICROSTRAIN_GIT_VERSION_CLEAN})

    # Zip specific configuration
    # Build different zip packages for each target
    if(MSVC)
        set(CPACK_PACKAGE_FILE_NAME "${MIP_FILE_NAME_PREFIX}_MSVC_v${MSVC_TOOLSET_VERSION}")
    elseif(APPLE)
        set(CPACK_PACKAGE_FILE_NAME "${MIP_FILE_NAME_PREFIX}_OSX")
    elseif(UNIX)
        set(CPACK_PACKAGE_FILE_NAME "${MIP_FILE_NAME_PREFIX}_Linux")
    else()
        set(CPACK_PACKAGE_FILE_NAME "${MIP_FILE_NAME_PREFIX}")
    endif()

    set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

    # Package everything into a single group (includes all MIP and MicroStrain SDK files/libs in one package)
    set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)

    # Finally include cpack which should have taken all of the previous variables into consideration
    include(CPack)
endif()
