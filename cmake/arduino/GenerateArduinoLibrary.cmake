# In order to copy the headers, create a cmake script that will do the copying
#set(MSCL_STATIC_HEADER_CP_SCRIPT ${CMAKE_BINARY_DIR}/${LIB_NAME_STATIC}_$<CONFIG>_header_cp.cmake)
#set(MSCL_STATIC_OUTPUT_DIR ${MSCL_OUTPUT_DIR}/C++/Static/${MSCL_ARCH_NAME}/$<CONFIG>/${LIB_NAME})

file(COPY ${SOURCE_DIR}
     DESTINATION ${OUTPUT_DIR}/src
     FILES_MATCHING PATTERN *.h PATTERN)

# Generate the library.properties file for Arduino library configuration
file(CONFIGURE
     OUTPUT "${OUTPUT_DIR}/library.properties"
     CONTENT "name=${LIBRARY_NAME}
version=${VERSION}
author=Microstrain
maintainer=Microstrain
sentence=${DESCRIPTION}
paragraph=${DESCRIPTION}
category=Communication
url=${URL}
architectures=avr
includes=mip_all.h
precompiled=true
ldflags=-l${LIBRARY_NAME}"
     )
