/////////////////////////////////////////////////////////////////////////////
//
// mip_binary_to_csv_converter.cpp
//
// C++ Example MIP binary file parsing program
//
// This example shows how to parse MIP binary files and convert them to CSV.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, PARKER MICROSTRAIN SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <experimental/filesystem>

#include "mip/mip_all.hpp"

#include "string_utilities.h"

const size_t MAX_BUFFER_SIZE = 1024;


struct PacketBuffer
{
    std::map<uint8_t,std::vector<mip::PacketBuf>> packet_buffer;

    bool handle_packet(const mip::PacketRef& packet, mip::Timestamp timestamp)
    {
        mip::PacketBuf packet_copy(packet);

        auto it = packet_buffer.find(packet_copy.descriptorSet());
        if (it == packet_buffer.end())
            packet_buffer[packet_copy.descriptorSet()] = std::vector<mip::PacketBuf>{packet_copy};
        else
            it->second.emplace_back(packet_copy);

        return true;
    }
};

void add_csv_row(std::string& csv_data, const std::vector<std::string>& row)
{
    // Combine all string elements into single comma seperated string
    for (const auto& val : row)
    {
        csv_data += val;
        csv_data += ",";
    }

    if (!csv_data.empty())
        csv_data.pop_back(); // Remove last comma from row;

    csv_data += "\n";
}

std::string convert_packets_to_csv(const std::vector<mip::PacketBuf>& packets)
{

    // Get field length for all descriptors
    std::map<uint8_t, size_t> field_length_map;
    for (const auto& packet : packets)
    {
        for (const auto& field : packet)
        {
            if (field_length_map.find(field.fieldDescriptor()) != field_length_map.end())
                continue;

            size_t field_length = get_field_length(field);
            if (field_length > 0)
                field_length_map[field.fieldDescriptor()] = field_length;
        }
    }

    // Get column offset indices for each field
    size_t num_columns = 0;
    std::map<uint8_t, size_t> field_offset_map;
    for (const auto& pair : field_length_map)
    {
        field_offset_map[pair.first] = num_columns;
        num_columns += pair.second;
    }

    if (num_columns == 0)
        return {};

    std::string csv_output;

    // Build column header
    std::vector<std::string> column_headers(num_columns, "");
    for (const auto& pair : field_offset_map)
    {
        uint8_t field_length = field_length_map[pair.first];
        size_t field_offset = field_offset_map[pair.first];
        for (int i = 0; i<field_length; i++)
            column_headers[field_offset + i] = get_column_header_string(pair.first, i);
    }
    // Add column header to CSV data
    add_csv_row(csv_output, column_headers);

    // Build rows
    for (const auto& packet : packets)
    {
        std::vector<std::string> row(num_columns, "");
        int values_added = 0;
        for (const auto& field : packet)
        {
            size_t field_offset = field_offset_map[field.fieldDescriptor()];
            std::vector<std::string> field_string = get_field_as_string(field);
            if (field_string.empty())
                continue;
            for (const auto& value : field_string)
            {
                row[field_offset++] = value;
                values_added++;
            }
        }

        // Check for empty row
        bool row_empty = values_added == 0;
        if (row_empty)
            continue;

        // Add row to CSV
        add_csv_row(csv_output, row);
    }

    return csv_output;
}


int main(int argc, char **argv)
{
    std::experimental::filesystem::path input_filepath;
    std::experimental::filesystem::path output_folderpath;

    // Parse input arguments
    if (argc == 2)
    {
        input_filepath = std::experimental::filesystem::path(argv[1]);
        output_folderpath = input_filepath.parent_path();
    }
    else if (argc == 3)
    {
        input_filepath = std::experimental::filesystem::path(argv[1]);
        output_folderpath = std::experimental::filesystem::path(argv[2]);
    }
    else
    {
        std::cerr << "Usage: <INPUT_BINARY_FILEPATH> [optional]<OUTPUT_FOLDERPATH>" << std::endl;
        return 0;
    }

    // Open binary file
    std::ifstream input(input_filepath.string(), std::ifstream::binary);
    if (!input)
    {
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }

    // Initialize data buffers
    char input_file_buffer[MAX_BUFFER_SIZE];
    uint8_t parser_buffer[MAX_BUFFER_SIZE];
    PacketBuffer packet_buffer;

    // Initialize parser
    mip::Parser mip_parser(parser_buffer, MAX_BUFFER_SIZE, MIPPARSER_DEFAULT_TIMEOUT_MS);
    mip_parser.setCallback<PacketBuffer, &PacketBuffer::handle_packet>(packet_buffer);

    // Parse data from file
    std::cout << "Loading data...";
    while (input)
    {
        input.read(input_file_buffer, MAX_BUFFER_SIZE);
        size_t bytes_read = input.gcount();
        mip_parser.parse(reinterpret_cast<const uint8_t*>(input_file_buffer), bytes_read, 0, MIPPARSER_UNLIMITED_PACKETS);
    }
    std::cout << "Done" << std::endl;

    // Build CSVs
    for (const auto& pair : packet_buffer.packet_buffer)
    {
        auto it = DESCRIPTOR_SET_NAME_MAP.find(pair.first);
        if (it == DESCRIPTOR_SET_NAME_MAP.end())
            continue;

        std::cout << "Building " << it->second << " data CSV file...";

        // Get output CSV filepath
        std::string output_csv_filename = input_filepath.filename().replace_extension("").string() + "_" + it->second + "_data.csv";
        std::experimental::filesystem::path output_csv_filepath = output_folderpath;
        output_csv_filepath.append(output_csv_filename);

        // Build output CSV string
        std::string csv_string = convert_packets_to_csv(pair.second);

        // Write CSV string to file
        std::ofstream out(output_csv_filepath.string());
        out << csv_string;
        out.close();

        std::cout << "Done" << std::endl;
    }

    return 0;
}