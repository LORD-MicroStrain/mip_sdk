#pragma once

#include <mip/definitions/data_filter.h>

#include <microstrain/connections/serial/serial_port.h>
#include <microstrain/logging.h>

extern serial_port device_port;

void mip_example_init();
void mip_example_log(void* user, const microstrain_log_level level, const char* fmt, va_list args);
mip_timestamp get_current_timestamp();
bool mip_interface_user_recv_from_device(mip_interface* device_, uint8_t* buffer, size_t max_length, mip_timeout wait_time, bool from_cmd, size_t* length_out, mip_timestamp* timestamp_out);
bool mip_interface_user_send_to_device(mip_interface* device_, const uint8_t* data, size_t length);

void exit_gracefully(const char *message);

// Displays current filter state for the connected device if it has changed.
void displayFilterState(const mip_filter_mode filterState, char **currentState, bool isFiveSeries);
