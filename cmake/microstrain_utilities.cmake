# Install the headers in their
macro(microstrain_setup_install_headers LIBRARY ROOT_DIR)
    include(GNUInstallDirs)

    # Only install headers that we build the source files for
    get_target_property(ALL_HEADERS ${LIBRARY} SOURCES)
    list(FILTER ALL_HEADERS INCLUDE REGEX "^.*\.(h|hpp)$")

    # Setup the include directory for each header within a MicroStrain subdirectory
    foreach(HEADER ${ALL_HEADERS})
        # Replace the source directory with the installer include directory (within a MicroStrain subdirectory)
        string(REPLACE
            "${ROOT_DIR}"
            "${CMAKE_INSTALL_INCLUDEDIR}/microstrain"
            HEADER_DESTINATION_FULL
            "${HEADER}"
        )

        # Get the install directory without the file
        get_filename_component(HEADER_DESTINATION "${HEADER_DESTINATION_FULL}" DIRECTORY)

        install(
            FILES "${HEADER}"
            DESTINATION "${HEADER_DESTINATION}"
        )
    endforeach()
endmacro()

set(MICROSTRAIN_CONFIG_FILE_IN ${CMAKE_CURRENT_LIST_DIR}/mip-config.cmake.in)

function(microstrain_generate_package_config PACKAGE_NAME)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    set(MICROSTRAIN_CONFIG_FILE_NAME "${PACKAGE_NAME}-config")

    set(MICROSTRAIN_VERSION_FILE_NAME "${MICROSTRAIN_CONFIG_FILE_NAME}-version.cmake")
    string(APPEND MICROSTRAIN_CONFIG_FILE_NAME ".cmake")
    set(MICROSTRAIN_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${MICROSTRAIN_CONFIG_FILE_NAME}")
    set(MICROSTRAIN_VERSION_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${MICROSTRAIN_VERSION_FILE_NAME}")

    set(MICROSTRAIN_SUBDIR_NAME "microstrain")

    if(WIN32)
        set(MICROSTRAIN_CMAKE_CONFIG_INSTALL_DIR "${PACKAGE_NAME}/cmake")
    else()
        set(MICROSTRAIN_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/cmake/${PACKAGE_NAME}")
    endif()

    set(INCLUDE_INSTALL_DIR_C "${CMAKE_INSTALL_INCLUDEDIR}/${MICROSTRAIN_SUBDIR_NAME}/c")
    set(INCLUDE_INSTALL_DIR_CPP "${CMAKE_INSTALL_INCLUDEDIR}/${MICROSTRAIN_SUBDIR_NAME}/cpp")
    set(SYSCONFIG_INSTALL_DIR "${CMAKE_INSTALL_SYSCONFDIR}/${MICROSTRAIN_SUBDIR_NAME}")
    set(LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")

    set(PACKAGE_VERSION "${MICROSTRAIN_GIT_VERSION_CLEAN}")
    string(TOUPPER "${PACKAGE_NAME}" PACKAGE_NAME_UPPER)

    configure_package_config_file(
        "${MICROSTRAIN_CONFIG_FILE_IN}"
        "${MICROSTRAIN_CONFIG_FILE_OUTPUT}"
        INSTALL_DESTINATION "${MICROSTRAIN_CMAKE_CONFIG_INSTALL_DIR}"
        PATH_VARS
            INCLUDE_INSTALL_DIR_C
            INCLUDE_INSTALL_DIR_CPP
            SYSCONFIG_INSTALL_DIR
            LIBRARY_INSTALL_DIR
    )

    write_basic_package_version_file(
        "${MICROSTRAIN_VERSION_FILE_OUTPUT}"
        VERSION "${MICROSTRAIN_GIT_VERSION_CLEAN}"
        COMPATIBILITY AnyNewerVersion
    )

    install(
        FILES
            "${MICROSTRAIN_CONFIG_FILE_OUTPUT}"
            "${MICROSTRAIN_VERSION_FILE_OUTPUT}"
        DESTINATION "${MICROSTRAIN_CMAKE_CONFIG_INSTALL_DIR}"
        COMPONENT "${PACKAGE_NAME}"
    )
endfunction()

macro(microstrain_setup_library_install LIBRARY ROOT_DIR)
    include(GNUInstallDirs)
    install(
        TARGETS ${LIBRARY}
        COMPONENT ${LIBRARY}
        EXPORT ${LIBRARY}-targets
        ARCHIVE
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME
            DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    microstrain_setup_install_headers(${LIBRARY} ${ROOT_DIR})
    microstrain_generate_package_config(${LIBRARY})
endmacro()

#
# Utility macros to write to output variables
# Do not call these in the conventional way I.E. macro_name(${VARIABLE_NAME})
# Instead, do macro_name(VARIABLE_NAME) which will do `set(${PARAM_OUT} value)` and is equivalent to set(VARIABLE_NAME value)
#

macro(microstrain_get_git_version GIT_VERSION_OUT GIT_VERSION_CLEAN_OUT)
    # Use Git to find the version
    if(NOT GIT_FOUND)
        find_package(Git)
    endif()

    set(MICROSTRAIN_DEFAULT_GIT_VERSION "v${PROJECT_VERSION}")

    if(NOT GIT_FOUND)
        message(WARNING "Unable to find Git, defaulting to version ${MICROSTRAIN_DEFAULT_GIT_VERSION}")
        set(${GIT_VERSION_OUT} ${MICROSTRAIN_DEFAULT_GIT_VERSION})
    else()
        # Find the latest semantic version tag I.E. 'v1.0.0' not 'latest'
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} describe --tags --match "v*"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE MICROSTRAIN_GIT_VERSION_OUT
            ERROR_VARIABLE MICROSTRAIN_GIT_VERSION_ERR
            RESULT_VARIABLE MICROSTRAIN_GIT_VERSION_RET
        )
        if(NOT ${MICROSTRAIN_GIT_VERSION_RET} EQUAL 0)
            message(STATUS "Unable to determine version from Git, defaulting to version ${MICROSTRAIN_DEFAULT_GIT_VERSION}")
            set(${GIT_VERSION_OUT} ${MICROSTRAIN_DEFAULT_GIT_VERSION})
        else()
            if("${MICROSTRAIN_GIT_VERSION_OUT}" MATCHES "^v.+")
                set(${GIT_VERSION_OUT} ${MICROSTRAIN_GIT_VERSION_OUT})
            else()
                message(STATUS "Unable to determine semantic version from Git, defaulting to version ${MICROSTRAIN_DEFAULT_GIT_VERSION}")
                set(${GIT_VERSION_OUT} ${MICROSTRAIN_DEFAULT_GIT_VERSION})
            endif()
            string(REGEX REPLACE "\n" "" ${GIT_VERSION_OUT} "${${GIT_VERSION_OUT}}")
            message(STATUS "Determined version from Git: ${${GIT_VERSION_OUT}}")
        endif()
    endif()


    # Massage the version number a little so we can use it in a couple places
    string(REGEX REPLACE "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+).*" "\\1.\\2.\\3" ${GIT_VERSION_CLEAN_OUT} ${${GIT_VERSION_OUT}})
endmacro()

# Use the variable name not value for GIT_VERSION_CLEAN to lookup the version and also set it
# This is a followup macro for microstrain_get_git_version
# I.E. microstrain_extract_git_version(MICROSTRAIN_GIT_VERSION_CLEAN MICROSTRAIN_GIT_VERSION_MAJOR ...)
macro(microstrain_extract_git_version GIT_VERSION_CLEAN GIT_VERSION_MAJOR_OUT GIT_VERSION_MINOR_OUT GIT_VERSION_PATCH_OUT)
    if(NOT DEFINED ${GIT_VERSION_CLEAN})
        message(WARNING "Use the name to use for the variable GIT_VERSION_CLEAN instead of passing the value")
    endif()

    string(REPLACE "." ";" MICROSTRAIN_GIT_VERSION_LIST ${${GIT_VERSION_CLEAN}})
    list(LENGTH MICROSTRAIN_GIT_VERSION_LIST MICROSTRAIN_GIT_VERSION_LIST_LENGTH)

    if(MICROSTRAIN_GIT_VERSION_LIST_LENGTH GREATER_EQUAL 3)
        list(GET MICROSTRAIN_GIT_VERSION_LIST 0 ${GIT_VERSION_MAJOR_OUT})
        list(GET MICROSTRAIN_GIT_VERSION_LIST 1 ${GIT_VERSION_MINOR_OUT})
        list(GET MICROSTRAIN_GIT_VERSION_LIST 2 ${GIT_VERSION_PATCH_OUT})
    else()
        message(WARNING "Version cannot be parsed into a semantic version string.\nPlease run 'git fetch --tags' to get a properly tagged build")
        set(${GIT_VERSION_CLEAN} "0.0.0")
        set(${GIT_VERSION_MAJOR_OUT} 0)
        set(${GIT_VERSION_MINOR_OUT} 0)
        set(${GIT_VERSION_PATCH_OUT} 0)
    endif()
endmacro()

# Try to determine what architecture we are building for based on the compiler output
# Specify the variable to set as the parameter
macro(microstrain_get_architecture SYS_ARCH_OUT)
    if(MSVC)
        # Detect if this is a x64 or x86 build
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(${SYS_ARCH_OUT} "x64")
        elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
            set(${SYS_ARCH_OUT} "x86")
        endif()
    elseif(UNIX)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            # The dumpmachine command from gcc should contain information on the architecture
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "${CMAKE_CXX_COMPILER} -dumpmachine"
                OUTPUT_VARIABLE MICROSTRAIN_GCC_ARCHITECTURE_OUT
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                ERROR_VARIABLE MICROSTRAIN_GCC_ARCHITECTURE_ERR
                RESULT_VARIABLE MICROSTRAIN_GCC_ARCHITECTURE_RET
            )

            # Convert the GCC architecture to the format that we use
            if("${MICROSTRAIN_GCC_ARCHITECTURE_OUT}" MATCHES ".*x86_64.*")
                set(${SYS_ARCH_OUT} "amd64")
            elseif("${MICROSTRAIN_GCC_ARCHITECTURE_OUT}" MATCHES ".*aarch64.*")
                set(${SYS_ARCH_OUT} "arm64")
            elseif("${MICROSTRAIN_GCC_ARCHITECTURE_OUT}" MATCHES ".*arm.*")
                set(${SYS_ARCH_OUT} "armhf")
            else()
                message(STATUS "Unrecognized GCC architecture ${MICROSTRAIN_GCC_ARCHITECTURE_OUT}. Using CMAKE_SYSTEM_PROCESSOR for architecture")
            endif()
        endif()
    endif()

    if(NOT DEFINED ${SYS_ARCH_OUT})
        message(STATUS "Defaulting ${SYS_ARCH_OUT} to ${CMAKE_SYSTEM_PROCESSOR}")
        set(${SYS_ARCH_OUT} ${CMAKE_SYSTEM_PROCESSOR})
    else()
        message(STATUS "Detected system architecture ${${SYS_ARCH_OUT}}")
    endif()
endmacro()
