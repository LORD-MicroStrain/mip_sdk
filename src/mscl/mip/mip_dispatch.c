
#include "mip_dispatch.h"

#include "mip_packet.h"
#include "mip_field.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
namespace C {
#endif


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the dispatch handler with a packet callback.
///
/// Packets which match the descriptor set will cause the callback to be
/// executed.
///
///@param handler
///
///@param descriptor_set
///       The callback will only be invoked for fields belonging to this
///       descriptor set. Can be MIP_DISPATCH_WILDCARD to match any packet,
///       or MIP_DISPATCH_DATA to match only data packets
///@param callback
///       The callback function.
///@param user_data
///       Any pointer the user wants to pass into the callback.
///
void mip_dispatch_handler_init_packet_handler(struct mip_dispatch_handler* handler, uint8_t descriptor_set, mip_dispatch_packet_callback callback, void* user_data)
{
    handler->next            = NULL;
    handler->packet_callback  = callback;
    handler->user_data        = user_data;

    handler->descriptor_set   = descriptor_set;
    handler->field_descriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    handler->enabled         = true;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the dispatch handler with a field callback.
///
/// Fields which match both the descriptor set and field descriptor will cause
/// the callback function to be executed.
///
///@param handler
///
///@param descriptor_set
///       The callback will only be invoked for fields belonging to this
///       descriptor set. Can be MIP_DISPATCH_DESCRIPTOR_WILDCARD to match any
///       packet, or MIP_DISPATCH_DESCSET_DATA to match only data packets.
///
///@param field_descriptor
///       The callback will only be invoked for fields of this field descriptor.
///       It can be MIP_DISPATCH_DESCRIPTOR_WILDCARD, but not
///       MIP_DISPATCH_FIELDDESC_NONE.
///
///@param callback
///       The callback function.
///
///@param user_data
///       Any pointer the user wants to pass into the callback.
///
void mip_dispatch_handler_init_field_handler(struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* user_data)
{
    assert(field_descriptor != MIP_DISPATCH_FIELDDESC_NONE);

    handler->next            = NULL;
    handler->field_callback   = callback;
    handler->user_data        = user_data;

    handler->descriptor_set   = descriptor_set;
    handler->field_descriptor = field_descriptor;
    handler->enabled         = true;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Enables or disables the handler.
///
///@param handler
///@param enable If true, the callback is enabled. If false, it will not be called.
///
void mip_dispatch_handler_set_enabled(struct mip_dispatch_handler* handler, bool enable)
{
    handler->enabled = enable;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the handler is currently enabled.
///
///@returns true if the handler is enabled, false otherwise.
///
bool mip_dispatch_handler_is_enabled(struct mip_dispatch_handler* handler)
{
    return handler->enabled;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the handler matches the given descriptor pair.
///
///@param handler
///@param descriptor_set
///@param field_descriptor
///
///@returns true if matching, false otherwise.
///
bool mip_dispatch_handler_matches(struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor)
{
    // First filter by descriptor set.
    switch(handler->descriptor_set)
    {
    case MIP_DISPATCH_DESCRIPTOR_WILDCARD:
        break;

    case MIP_DISPATCH_DESCSET_DATA:
        if( !is_data_descriptor_set(descriptor_set) )
            return false;
        break;

    default:
        if( descriptor_set != handler->descriptor_set )
            return false;
        break;
    }

    // Descriptor set check passed, now check field descriptor.
    // Packet callbacks must never be called for fields and vice versa!
    if( field_descriptor == MIP_DISPATCH_FIELDDESC_NONE )
    {
        // This is a packet callback check.
        // Match if and only if the handler is also a packet callback.
        return handler->field_descriptor == MIP_DISPATCH_FIELDDESC_NONE;
    }
    else
    {
        // Field callback - check for wildcard or exact match.
        return (handler->field_descriptor == MIP_DISPATCH_DESCRIPTOR_WILDCARD) || (field_descriptor == handler->field_descriptor);
    }
}

////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the mip_dispatcher object.
///
void mip_dispatcher_init(struct mip_dispatcher* self)
{
    self->first_handler = NULL;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Registers a handler in the dispatch system.
///
/// This is necessary for the handler function to be executed.
///
void mip_dispatcher_add_handler(struct mip_dispatcher* self, struct mip_dispatch_handler* handler)
{
    if( self->first_handler == NULL )
        self->first_handler = handler;
    else
    {
        struct mip_dispatch_handler* last = self->first_handler;

        while(last->next != NULL)
            last = last->next;

        last->next = handler;
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Removes a handler from the dispatch system.
///
/// This will prevent the handler from being executed.
///
void mip_dispatcher_remove_handler(struct mip_dispatcher* self, struct mip_dispatch_handler* handler)
{
    if( self->first_handler == NULL )
        return;

    struct mip_dispatch_handler* query = self->first_handler;

    if( query == handler )
    {
        self->first_handler = handler->next;
        handler->next = NULL;
        return;
    }

    while(query->next != NULL)
    {
        if( query->next == handler )
        {
            query->next = handler->next;
            handler->next = NULL;
            return;
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Removes all handlers from the dispatcher.
///
void mip_dispatcher_remove_all_handlers(struct mip_dispatcher* self)
{
    struct mip_dispatch_handler* query = self->first_handler;

    self->first_handler = NULL;

    // Break the chain (technically not necessary, but aids debugging)
    while(query)
    {
        struct mip_dispatch_handler* next = query->next;
        query->next = NULL;
        query = next;
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Called to dispatch the callbacks for a given packet.
///
///@param self
///
///@param packet
///       The MIP packet to dispatch.
///
///@param timestamp
///        The approximate parse time of the packet.
///
void mip_dispatcher_dispatch_packet(struct mip_dispatcher* self, const struct mip_packet* packet, timestamp_type timestamp)
{
    const uint8_t descriptor_set = mip_packet_descriptor_set(packet);

    struct mip_dispatch_handler* handler;

    // Iterate all packet handlers for this packet.
    for(handler = self->first_handler; handler != NULL; handler = handler->next)
    {
        if( mip_dispatch_handler_matches(handler, descriptor_set, MIP_INVALID_FIELD_DESCRIPTOR) )
            handler->packet_callback(handler->user_data, packet, timestamp);
    }

    for(struct mip_field field = mip_field_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field))
    {
        const uint8_t field_descriptor = mip_field_field_descriptor(&field);

        // Iterate all field handlers for this field.
        for(handler = self->first_handler; handler != NULL; handler = handler->next)
        {
            if( mip_dispatch_handler_matches(handler, descriptor_set, field_descriptor) )
            {
                handler->field_callback(handler->user_data, &field, timestamp);
                break;
            }
        };
    }
}

#ifdef __cplusplus
} // namespace C
} // namespace mscl
#endif
