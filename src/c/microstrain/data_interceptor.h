#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef void (*microstrain_interceptor_data_point_callback)(void* _data, void* _context);
typedef void (*microstrain_interceptor_array_callback)(void* _data, const size_t _data_length, void* _context);

typedef enum microstrain_interceptor_type
{
    MICROSTRAIN_INTERCEPTOR_SINGLE,
    MICROSTRAIN_INTERCEPTOR_ARRAY
} microstrain_interceptor_type;

typedef struct microstrain_interceptor_handler
{
    /// @brief Callbacks based on the data type
    union callback
    {
        /// @ brief Single data quantity callback
        microstrain_interceptor_data_point_callback data_point_callback;

        /// @brief Data array callback
        microstrain_interceptor_array_callback array_callback;
    } callbacks;

    /// @brief Data context. Can be NULL
    void* context;

    /// @brief Pointer to the next handler in the list
    struct microstrain_interceptor_handler* next;

    microstrain_interceptor_type type;
} microstrain_interceptor_handler;

void microstrain_interceptor_handler_init_data_point_handler(microstrain_interceptor_handler* _handler,
    const microstrain_interceptor_data_point_callback _callback, void* _context);

void microstrain_interceptor_handler_init_array_data_handler(microstrain_interceptor_handler* _handler,
    const microstrain_interceptor_array_callback _callback, void* _context);

typedef struct microstrain_interceptor
{
    /// @brief Pointer to the first interceptor handler
    microstrain_interceptor_handler* head;
} microstrain_interceptor;

void microstrain_interceptor_init(microstrain_interceptor* _interceptor);

void microstrain_interceptor_add_handler(microstrain_interceptor* _interceptor,
    microstrain_interceptor_handler* _handler);

void microstrain_interceptor_remove_handler(microstrain_interceptor* _interceptor,
    microstrain_interceptor_handler* _handler);

void microstrain_interceptor_remove_all_handlers(microstrain_interceptor* _interceptor);

void microstrain_interceptor_dispatch_data_point(const microstrain_interceptor* _interceptor, void* _data);

void microstrain_interceptor_dispatch_array_data(const microstrain_interceptor* _interceptor, void* _data,
    const size_t _data_length);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
