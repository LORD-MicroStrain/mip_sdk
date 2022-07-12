
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
///@param descriptorSet
///       The callback will only be invoked for fields belonging to this
///       descriptor set. Can be MIP_DISPATCH_WILDCARD to match any packet,
///       or MIP_DISPATCH_DATA to match only data packets
///@param callback
///       The callback function.
///@param userData
///       Any pointer the user wants to pass into the callback.
///
void MipDispatchHandler_initPacketHandler(struct MipDispatchHandler* handler, uint8_t descriptorSet, MipDispatchPacketCallback callback, void* userData)
{
    handler->next            = NULL;
    handler->packetCallback  = callback;
    handler->userData        = userData;

    handler->descriptorSet   = descriptorSet;
    handler->fieldDescriptor = MIP_INVALID_FIELD_DESCRIPTOR;
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
///@param descriptorSet
///       The callback will only be invoked for fields belonging to this
///       descriptor set. Can be MIP_DISPATCH_DESCRIPTOR_WILDCARD to match any
///       packet, or MIP_DISPATCH_DESCSET_DATA to match only data packets.
///
///@param fieldDescriptor
///       The callback will only be invoked for fields of this field descriptor.
///       It can be MIP_DISPATCH_DESCRIPTOR_WILDCARD, but not
///       MIP_DISPATCH_FIELDDESC_NONE.
///
///@param callback
///       The callback function.
///
///@param userData
///       Any pointer the user wants to pass into the callback.
///
void MipDispatchHandler_initFieldHandler(struct MipDispatchHandler* handler, uint8_t descriptorSet, uint8_t fieldDescriptor, MipDispatchFieldCallback callback, void* userData)
{
    assert(fieldDescriptor != MIP_DISPATCH_FIELDDESC_NONE);

    handler->next            = NULL;
    handler->fieldCallback   = callback;
    handler->userData        = userData;

    handler->descriptorSet   = descriptorSet;
    handler->fieldDescriptor = fieldDescriptor;
    handler->enabled         = true;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Enables or disables the handler.
///
///@param handler
///@param enable If true, the callback is enabled. If false, it will not be called.
///
void MipDispatchHandler_setEnabled(struct MipDispatchHandler* handler, bool enable)
{
    handler->enabled = enable;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the handler is currently enabled.
///
///@returns true if the handler is enabled, false otherwise.
///
bool MipDispatchHandler_isEnabled(struct MipDispatchHandler* handler)
{
    return handler->enabled;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the handler matches the given descriptor pair.
///
///@param handler
///@param descriptorSet
///@param fieldDescriptor
///
///@returns true if matching, false otherwise.
///
bool MipDispatchHandler_matches(struct MipDispatchHandler* handler, uint8_t descriptorSet, uint8_t fieldDescriptor)
{
    // First filter by descriptor set.
    switch(handler->descriptorSet)
    {
    case MIP_DISPATCH_DESCRIPTOR_WILDCARD:
        break;

    case MIP_DISPATCH_DESCSET_DATA:
        if( !isDataDescriptorSet(descriptorSet) )
            return false;
        break;

    default:
        if( descriptorSet != handler->descriptorSet )
            return false;
        break;
    }

    // Descriptor set check passed, now check field descriptor.
    // Packet callbacks must never be called for fields and vice versa!
    if( fieldDescriptor == MIP_DISPATCH_FIELDDESC_NONE )
    {
        // This is a packet callback check.
        // Match if and only if the handler is also a packet callback.
        return handler->fieldDescriptor == MIP_DISPATCH_FIELDDESC_NONE;
    }
    else
    {
        // Field callback - check for wildcard or exact match.
        return (handler->fieldDescriptor == MIP_DISPATCH_DESCRIPTOR_WILDCARD) || (fieldDescriptor == handler->fieldDescriptor);
    }
}

////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the MipDispatcher object.
///
void MipDispatcher_init(struct MipDispatcher* self)
{
    self->firstHandler = NULL;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Registers a handler in the dispatch system.
///
/// This is necessary for the handler function to be executed.
///
void MipDispatcher_addHandler(struct MipDispatcher* self, struct MipDispatchHandler* handler)
{
    if( self->firstHandler == NULL )
        self->firstHandler = handler;
    else
    {
        struct MipDispatchHandler* last = self->firstHandler;

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
void MipDispatcher_removeHandler(struct MipDispatcher* self, struct MipDispatchHandler* handler)
{
    if( self->firstHandler == NULL )
        return;

    struct MipDispatchHandler* query = self->firstHandler;

    if( query == handler )
    {
        self->firstHandler = handler->next;
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
void MipDispatcher_removeAllHandlers(struct MipDispatcher* self)
{
    struct MipDispatchHandler* query = self->firstHandler;

    self->firstHandler = NULL;

    // Break the chain (technically not necessary, but aids debugging)
    while(query)
    {
        struct MipDispatchHandler* next = query->next;
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
void MipDispatcher_dispatchPacket(struct MipDispatcher* self, const struct MipPacket* packet, Timestamp timestamp)
{
    const uint8_t descriptorSet = MipPacket_descriptorSet(packet);

    struct MipDispatchHandler* handler;

    // Iterate all packet handlers for this packet.
    for(handler = self->firstHandler; handler != NULL; handler = handler->next)
    {
        if( MipDispatchHandler_matches(handler, descriptorSet, MIP_INVALID_FIELD_DESCRIPTOR) )
            handler->packetCallback(handler->userData, packet, timestamp);
    }

    for(struct MipField field = MipField_fromPacket(packet); MipField_isValid(&field); MipField_next(&field))
    {
        const uint8_t fieldDescriptor = MipField_fieldDescriptor(&field);

        // Iterate all field handlers for this field.
        for(handler = self->firstHandler; handler != NULL; handler = handler->next)
        {
            if( MipDispatchHandler_matches(handler, descriptorSet, fieldDescriptor) )
            {
                handler->fieldCallback(handler->userData, &field, timestamp);
                break;
            }
        };
    }
}

//
// ////////////////////////////////////////////////////////////////////////////////
// ///@brief Finds the next handler for the given descriptor pair.
// ///
// ///@param self
// ///
// ///@param descriptorSet
// ///@param fieldDescriptor
// ///
// ///@param[in,out] iterator
// ///       Iterator over handles. The search starts with *handle, if not NULL.
// ///
// ///@returns true if a handler was found. This is the same as *handler != NULL.
// ///
// struct MipDispatchHandler* MipDispatcher_nextHandler(struct MipDispatcher* self, uint8_t descriptorSet, uint8_t fieldDescriptor, struct MipDispatchHandler* start)
// {
//     if(start == NULL)
//         start = self->firstHandler;
//     else
//         start = start->next;
//
//     while(start)
//     {
//         if( MipDispatchHandler_matches(start, descriptorSet, fieldDescriptor) )
//             break;
//     }
//
//     return start;
// }

#ifdef __cplusplus
} // namespace C
} // namespace mscl
#endif
