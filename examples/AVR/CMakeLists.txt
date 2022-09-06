cmake_minimum_required(VERSION 3.22)

project(Arduino_Example)

get_board_id(board_id mega atmega2560)

arduino_cmake_project(Mip BOARD_NAME mega BOARD_CPU atmega2560)

add_arduino_executable(Arduino_Example CV7_example.cpp)
add_arduino_executable(Arduino_Example_Sketch "")
target_sketches(Arduino_Example_Sketch AVR.ino)

add_library(mip_avr STATIC ${ALL_MIP_SOURCES})
add_dependencies(Arduino_Example mip_avr)
add_dependencies(Arduino_Example_Sketch mip_avr)

#add_arduino_header_only_library(MipSDK ${board_id} ${ALL_MIP_SOURCES})
#add_dependencies(Arduino_Example MipSDK)

#find_arduino_library(arduino_stl ArduinoSTL ${board_id} 3RD_PARTY)
#find_arduino_library(arx_type_traits ArxTypeTraits ${board_id} 3RD_PARTY HEADER_ONLY)
#find_arduino_library(chrono Chrono ${board_id} 3RD_PARTY HEADER_ONLY)

target_include_directories(Arduino_Example PRIVATE "${CMAKE_SOURCE_DIR}/src/mip")
target_include_directories(Arduino_Example_Sketch PRIVATE "${CMAKE_SOURCE_DIR}/src/mip")
#target_include_directories(Arduino_Example PRIVATE "${ARDUINO_CMAKE_SKETCHBOOK_PATH}/libraries/ArduinoSTL/src")
#target_include_directories(Arduino_Example PRIVATE "${ARDUINO_CMAKE_SKETCHBOOK_PATH}/libraries/ArxTypeTraits")
#target_include_directories(Arduino_Example PRIVATE "${ARDUINO_CMAKE_SKETCHBOOK_PATH}/libraries/Chrono")

#link_arduino_library(Arduino_Example MipSDK HEADER_ONLY)
link_arduino_library(Arduino_Example mip_avr ${board_id})
link_arduino_library(Arduino_Example_Sketch mip_avr ${board_id})
#link_arduino_library(Arduino_Example arduino_stl ${board_id})
#link_arduino_library(Arduino_Example arx_type_traits ${board_id} HEADER_ONLY)
#link_arduino_library(Arduino_Example chrono ${board_id} HEADER_ONLY)

set_target_upload_port(Arduino_Example COM8)
set_target_upload_port(Arduino_Example_Sketch COM8)
