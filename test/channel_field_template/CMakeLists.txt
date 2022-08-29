cmake_minimum_required(VERSION 3.22)
project(ChannelFieldRework)

set(CMAKE_CXX_STANDARD 11)

add_executable(ChannelFieldRework
               main.cpp
               ChannelField.cpp
               ChannelField.hpp
               Utils.cpp
               Utils.hpp
               )