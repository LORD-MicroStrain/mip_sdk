#pragma once

#include "microstrain/data_interceptor.h"

typedef struct microstrain_connection_data_interceptors
{
    microstrain_interceptor* receive_data_interceptor;
    microstrain_interceptor* send_data_interceptor;
} microstrain_connection_data_interceptors;

static void microstrain_connection_data_interceptors_set_interceptors(
    microstrain_connection_data_interceptors* _interceptors, microstrain_interceptor*
    _receive_interceptor, microstrain_interceptor* _send_interceptor)
{
    _interceptors->receive_data_interceptor = _receive_interceptor;
    _interceptors->send_data_interceptor    = _send_interceptor;
}

static void microstrain_connection_data_interceptors_init(microstrain_connection_data_interceptors* _interceptors)
{
    microstrain_connection_data_interceptors_set_interceptors(_interceptors, NULL, NULL);
}

static void microstrain_connection_data_interceptors_set_receive_interceptor(
    microstrain_connection_data_interceptors* _interceptors, microstrain_interceptor* _receive_interceptor)
{
    _interceptors->receive_data_interceptor = _receive_interceptor;
}

static void microstrain_connection_data_interceptors_set_send_interceptor(
    microstrain_connection_data_interceptors* _interceptors, microstrain_interceptor* _send_interceptor)
{
    _interceptors->send_data_interceptor = _send_interceptor;
}
