
#pragma once

#ifdef MICROSTRAIN_ENABLE_SERIAL
    #include "microstrain/connections/serial/serial_connection.hpp"
#endif
#ifdef MICROSTRAIN_ENABLE_TCP
    #include "microstrain/connections/tcp/tcp_connection.hpp"
#endif
#if defined MICROSTRAIN_ENABLE_SERIAL || defined MICROSTRAIN_ENABLE_TCP
    #include "microstrain/connections/recording/recording_connection.hpp"
#endif


#include <mip/mip_interface.hpp>
#include <mip/definitions/data_filter.hpp>

#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <exception>

struct ExampleUtils
{
    std::unique_ptr<microstrain::Connection> connection;
    std::unique_ptr<mip::Interface> device;
    std::unique_ptr<std::ofstream> recordedFile;
};

mip::Timestamp getCurrentTimestamp();

std::unique_ptr<ExampleUtils> openFromArgs(const std::string& port_or_hostname, const std::string& baud_or_port, const std::string& binary_file_path);

std::unique_ptr<ExampleUtils> handleCommonArgs(int argc, const char* argv[], int maxArgs=4);
int printCommonUsage(const char* argv[]);

/// Displays current filter state for the connected device if it has changed.
void displayFilterState(const mip::data_filter::FilterMode &filterState, std::string &currentState, bool isFiveSeries = false);
