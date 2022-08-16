#pragma once

#include "mip_types.h"
#include "definitions/descriptors.h"

#include <stdbool.h>


#ifdef __cplusplus
namespace mip {
namespace C {
#endif

struct mip_packet;
struct mip_field;


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipDispatch System for issuing callbacks from MIP packets or fields
///
///@{



////////////////////////////////////////////////////////////////////////////////
///@brief Signature for packet-level callbacks.
///
///@param context   User-supplied data pointer.
///@param packet    The MIP packet triggering this callback.
///@param timestamp The approximate parse time of the packet.
///
typedef void (*mip_dispatch_packet_callback)(void* context, const struct mip_packet* packet, timestamp_type timestamp);

////////////////////////////////////////////////////////////////////////////////
///@brief Signature for field-level callbacks.
///
///@param context   User-supplied data pointer.
///@param field     The MIP field triggering this callback.
///@param timestamp The approximate parse time of the packet.
///
typedef void (*mip_dispatch_field_callback )(void* context, const struct mip_field* field, timestamp_type timestamp);


enum {
    /// Field descriptor for packet callbacks. Never matches a field descriptor.
    /// 0x00 is not a valid MIP field descriptor.
    MIP_DISPATCH_FIELDDESC_NONE      = 0x00,

    /// Descriptor set for packet callbacks similar to the wildcard but
    /// only processing data packets.
    /// 0x00 is not a valid MIP descriptor set.
    MIP_DISPATCH_DESCSET_DATA        = 0x00,

    /// Descriptor set wildcard, meaning any descriptor set, or field descriptor
    /// wildcard, meaning any field within the descriptor set.
    /// 0xFF is not a valid MIP descriptor.
    MIP_DISPATCH_DESCRIPTOR_WILDCARD = 0xFF,
};

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_dispatch_handler mip_dispatch_handler
///
/// This represents a binding between a MIP descriptor pair and a callback
/// function.
///
/// This object must be valid for the duration of its registration in a
/// mip_dispatcher. It cannot be reinitialized while registered.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
///@{



///@brief Handler information for MIP Packet or Field callbacks.
///
struct mip_dispatch_handler
{
    struct mip_dispatch_handler* _next;                ///<@private Pointer to the next handler in the list.
    union
    {
        mip_dispatch_packet_callback _packet_callback;  ///<@private User function for packets. Valid if field_descriptor == 0x00.
        mip_dispatch_field_callback  _field_callback;   ///<@private User callback for data fields. Valid if field_descriptor != 0x00.
    };
    void*   _user_data;                                 ///<@private User-provided pointer which is passed directly to the callback.
    uint8_t _descriptor_set;                            ///<@private MIP descriptor set for this callback.
    uint8_t _field_descriptor;                          ///<@private MIP field descriptor for this callback. If 0x00, the callback is a packet callback.
    bool    _enabled;                                  ///<@private If false, the handler will be ignored.
};


void mip_dispatch_handler_init_packet_handler(struct mip_dispatch_handler* handler, uint8_t descriptor_set, mip_dispatch_packet_callback callback, void* context);
void mip_dispatch_handler_init_field_handler(struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* context);

void mip_dispatch_handler_set_enabled(struct mip_dispatch_handler* handler, bool enable);
bool mip_dispatch_handler_is_enabled(struct mip_dispatch_handler* handler);

bool mip_dispatch_handler_matches(struct mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor);


///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup MipDispatchHandler mip_dispatch_handler - Represents a callback
///
///@{


///@brief Holds the state of the MIP dispatch system.
///
struct mip_dispatcher
{
    struct mip_dispatch_handler* _first_handler;   ///<@private Pointer to the first dispatch handler. May be NULL.
};


void mip_dispatcher_init(struct mip_dispatcher* self);
void mip_dispatcher_add_handler(struct mip_dispatcher* self, struct mip_dispatch_handler* handler);
void mip_dispatcher_remove_handler(struct mip_dispatcher* self, struct mip_dispatch_handler* handler);
void mip_dispatcher_remove_all_handlers(struct mip_dispatcher* self);

void mip_dispatcher_dispatch_packet(struct mip_dispatcher* self, const struct mip_packet* packet, timestamp_type timestamp);

///@}
///@}
////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
} // namespace C
} // namespace mip
#endif
