#include "microstrain/data_interceptor.h"

#include <assert.h>

static void microstrain_init_interceptor_handler(microstrain_interceptor_handler* _handler, void* _context,
    const microstrain_interceptor_type _type)
{
    _handler->context = _context;
    _handler->next    = NULL;
    _handler->type    = _type;
}

void microstrain_interceptor_handler_init_data_point_handler(microstrain_interceptor_handler* _handler,
    const microstrain_interceptor_data_point_callback _callback, void* _context)
{
    assert(_callback);

    _handler->callbacks.data_point_callback = _callback;

    microstrain_init_interceptor_handler(_handler, _context, MICROSTRAIN_INTERCEPTOR_SINGLE);
}

void microstrain_interceptor_handler_init_array_data_handler(microstrain_interceptor_handler* _handler,
    const microstrain_interceptor_array_callback _callback, void* _context)
{
    assert(_callback);

    _handler->callbacks.array_callback = _callback;

    microstrain_init_interceptor_handler(_handler, _context, MICROSTRAIN_INTERCEPTOR_ARRAY);
}

void microstrain_interceptor_init(microstrain_interceptor* _interceptor)
{
    _interceptor->head = NULL;
}

static microstrain_interceptor_handler* microstrain_interceptor_find_previous_handler(
    const microstrain_interceptor* _interceptor, const microstrain_interceptor_handler* _handler)
{
    // No head or the handler is the head (no previous handler)
    if (_interceptor->head == NULL || _interceptor->head == _handler)
    {
        return NULL;
    }

    microstrain_interceptor_handler* previous = _interceptor->head;

    while (previous != NULL && previous->next != _handler)
    {
        previous = previous->next;
    }

    return previous;
}

static bool microstrain_interceptor_has_handler(const microstrain_interceptor* _interceptor,
    const microstrain_interceptor_handler* _handler, const microstrain_interceptor_handler* _previous_handler_out)
{
    _previous_handler_out = NULL;

    if (_interceptor->head == NULL)
    {
        return false;
    }

    if (_interceptor->head == _handler)
    {
        return true;
    }

    _previous_handler_out = microstrain_interceptor_find_previous_handler(_interceptor, _handler);

    return _previous_handler_out != NULL;
}

void microstrain_interceptor_add_handler(microstrain_interceptor* _interceptor,
    microstrain_interceptor_handler* _handler)
{
    assert(_handler->next == NULL);

    // First handler in the list
    if (_interceptor->head == NULL)
    {
        _interceptor->head = _handler;
        _handler->next     = NULL;

        return;
    }

    // The handler is already in the list
    if (microstrain_interceptor_has_handler(_interceptor, _handler, NULL))
    {
        return;
    }

    microstrain_interceptor_handler* last = _interceptor->head;

    // Find the last node in the list
    while (last->next != NULL)
    {
        last = last->next;
    }

    // Add the new handler and make sure it's the end of the list
    last->next = _handler;
    _handler->next = NULL;
}

void microstrain_interceptor_remove_handler(microstrain_interceptor* _interceptor,
    microstrain_interceptor_handler* _handler)
{
    // Set a new head and clear the link for the removed handler
    if (_interceptor->head == _handler)
    {
        _interceptor->head = _interceptor->head->next;
        _handler->next     = NULL;
        return;
    }

    microstrain_interceptor_handler* previous = NULL;

    if (!microstrain_interceptor_has_handler(_interceptor, _handler, previous))
    {
        return;
    }

    // The handler is the head
    if (previous == NULL)
    {
        _interceptor->head = _interceptor->head->next;
        _handler->next     = NULL;
        return;
    }

    // Close the link
    if (previous->next != NULL)
    {
        previous->next = _handler->next;
    }

    // Break the link on the removed handler
    _handler->next = NULL;
}

void microstrain_interceptor_remove_all_handlers(microstrain_interceptor* _interceptor)
{
    while (_interceptor->head != NULL)
    {
        microstrain_interceptor_handler* removed = _interceptor->head;
        _interceptor->head = removed->next;
        removed->next = NULL;
    }
}

void microstrain_interceptor_dispatch_data_point(const microstrain_interceptor* _interceptor, void* _data)
{
    const microstrain_interceptor_handler* node = _interceptor->head;

    while (node != NULL)
    {
        if (node->type == MICROSTRAIN_INTERCEPTOR_SINGLE && node->callbacks.data_point_callback != NULL)
        {
            node->callbacks.data_point_callback(_data, node->context);
        }

        node = node->next;
    }
}

void microstrain_interceptor_dispatch_array_data(const microstrain_interceptor* _interceptor, void* _data,
    const size_t _data_length)
{
    const microstrain_interceptor_handler* node = _interceptor->head;

    while (node != NULL)
    {
        if (node->type == MICROSTRAIN_INTERCEPTOR_ARRAY && node->callbacks.array_callback != NULL)
        {
            node->callbacks.array_callback(_data, _data_length, node->context);
        }

        node = node->next;
    }
}
