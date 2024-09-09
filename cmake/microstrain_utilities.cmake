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

macro(microstrain_setup_library_install LIBRARY ROOT_DIR)
    install(
        TARGETS ${LIBRARY}
        EXPORT ${LIBRARY}-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    microstrain_setup_install_headers(${LIBRARY} ${ROOT_DIR})

#    include(CMakePackageConfigHelpers)

#    set(CONFIG_EXPORT_DIR "${CMAKE_INSTALL_DATADIR}/cmake/${LIBRARY}")

    # TODO: Do we want this configure step? What is it used for?
#    install(
#        FILES "${CMAKE_BINARY_DIR}/${LIBRARY}-config.cmake" "${CMAKE_BINARY_DIR}/${LIBRARY}-config-version.cmake"
#        DESTINATION ${CONFIG_EXPORT_DIR}
#        COMPONENT ${LIBRARY}
#    )
endmacro()

#
# Utility macros to write to output variables
# Do not call these in the conventional way I.E. macro_name(${VARIABLE_NAME})
# Instead, do macro_name(VARIABLE_NAME) which will do `set(${PARAM_OUT} value)` and is equivalent to set(VARIABLE_NAME value)
#

macro(microstrain_get_git_version GIT_VERSION_OUT GIT_VERSION_CLEAN_OUT)
    # Use Git to find the version
    find_package(Git)

    set(MICROSTRAIN_DEFAULT_GIT_VERSION "v0.0.0")

    if(NOT GIT_FOUND)
        message(WARNING "Unable to find Git, will build with unknown version")
        set(${GIT_VERSION_OUT} ${MICROSTRAIN_DEFAULT_GIT_VERSION})
    else()
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} describe --tags
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            OUTPUT_VARIABLE MICROSTRAIN_GIT_VERSION_OUT
            ERROR_VARIABLE MICROSTRAIN_GIT_VERSION_ERR
            RESULT_VARIABLE MICROSTRAIN_GIT_VERSION_RET
        )
        if(NOT ${MICROSTRAIN_GIT_VERSION_RET} EQUAL 0)
            message(STATUS "Unable to determine version from Git, defaulting to version ${MICROSTRAIN_DEFAULT_GIT_VERSION}")
            set(${GIT_VERSION_OUT} ${MICROSTRAIN_DEFAULT_GIT_VERSION})
        else()
            set(${GIT_VERSION_OUT} ${MICROSTRAIN_GIT_VERSION_OUT})
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
