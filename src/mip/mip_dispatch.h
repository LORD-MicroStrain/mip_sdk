#pragma once

#include "../types.h"
#include "definitions/descriptors.h"

#include <stdbool.h>


#ifdef __cplusplus
namespace mscl {
namespace C {
#endif

struct MipPacket;
struct MipField;


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipDispatch System for issuing callbacks from MIP packets or fields
///
///@{



////////////////////////////////////////////////////////////////////////////////
///@brief Signature for packet-level callbacks.
///
///@param userData  User-supplied data pointer.
///@param packet    The MIP packet triggering this callback.
///@param timestamp The approximate parse time of the packet.
///
typedef void (*MipDispatchPacketCallback)(void* userData, const struct MipPacket* packet, Timestamp timestamp);

////////////////////////////////////////////////////////////////////////////////
///@brief Signature for field-level callbacks.
///
///@param userData  User-supplied data pointer.
///@param field     The MIP field triggering this callback.
///@param timestamp The approximate parse time of the packet.
///
typedef void (*MipDispatchFieldCallback )(void* userData, const struct MipField* field,   Timestamp timestamp);


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
///@defgroup MipDispatchHandler MipDispatchHandler
///
/// This represents a binding between a MIP descriptor pair and a callback
/// function.
///
/// This object must be valid for the duration of its registration in a
/// MipDispatcher. It cannot be reinitialized while registered.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
///@{



///@brief Handler information for MIP Packet or Field callbacks.
///
struct MipDispatchHandler
{
    struct MipDispatchHandler* next;               ///<@private Pointer to the next handler in the list.
    union
    {
        MipDispatchPacketCallback packetCallback;  ///<@private User function for packets. Valid if fieldDescriptor == 0x00.
        MipDispatchFieldCallback  fieldCallback;   ///<@private User callback for data fields. Valid if fieldDescriptor != 0x00.
    };
    void*   userData;                              ///<@private User-provided pointer which is passed directly to the callback.
    uint8_t descriptorSet;                         ///<@private MIP descriptor set for this callback.
    uint8_t fieldDescriptor;                       ///<@private MIP field descriptor for this callback. If 0x00, the callback is a packet callback.
    bool    enabled;                               ///<@private If false, the handler will be ignored.
};


void MipDispatchHandler_initPacketHandler(struct MipDispatchHandler* handler, uint8_t descriptorSet, MipDispatchPacketCallback callback, void* userData);
void MipDispatchHandler_initFieldHandler(struct MipDispatchHandler* handler, uint8_t descriptorSet, uint8_t fieldDescriptor, MipDispatchFieldCallback callback, void* userData);

void MipDispatchHandler_setEnabled(struct MipDispatchHandler* handler, bool enable);
bool MipDispatchHandler_isEnabled(struct MipDispatchHandler* handler);

bool MipDispatchHandler_matches(struct MipDispatchHandler* handler, uint8_t descriptorSet, uint8_t fieldDescriptor);


///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup MipDispatchHandler MipDispatchHandler - Represents a callback
///
///@{


///@brief Holds the state of the MIP dispatch system.
///
struct MipDispatcher
{
    struct MipDispatchHandler* firstHandler;   ///<@private Pointer to the first dispatch handler. May be NULL.
};


void MipDispatcher_init(struct MipDispatcher* self);
void MipDispatcher_addHandler(struct MipDispatcher* self, struct MipDispatchHandler* handler);
void MipDispatcher_removeHandler(struct MipDispatcher* self, struct MipDispatchHandler* handler);
void MipDispatcher_removeAllHandlers(struct MipDispatcher* self);

void MipDispatcher_dispatchPacket(struct MipDispatcher* self, const struct MipPacket* packet, Timestamp timestamp);

// bool MipDispatcher_nextHandler(struct MipDispatcher* self, uint8_t descriptorSet, uint8_t fieldDescriptor, struct MipDispatchHandler** handler);


///@}
////////////////////////////////////////////////////////////////////////////////




//
// void MipDispatchState_registerPacketCallback(struct MipDispatchState* state, uint8_t descriptorSet, MipDispatchPacketCallback callback, void* userData);
// void MipDispatchState_registerFieldCallback(struct MipDispatchState* state, uint8_t descriptorSet, uint8_t fieldDescriptor, MipDispatchFieldCallback callback, void* userData);
//
// void MipDispatchState_removeCallback(struct MipDispatchState* state, uint8_t descriptorSet, uint8_t fieldDescriptor);
//
// void MipDispatchState_dispatchPacket(struct MipDispatchState* state, const struct MipPacket* packet, Timestamp timestamp);
//
//
// void MipDispatchState_registerPacketCallback(
//     struct MipDispatchState* state, struct MipDispatchEntry*,
//     uint8_t descriptorSet, MipDispatchPacketCallback callback, void* userData);


// void MipDispatchHandler_setPacketCallback(struct MipDispatchHandler* handler, MipDispatchPacketCallback callback, void* userData);

// void foo()
// {
//     // MipDispatchState_init(&state, &device);
//     MipInterface_init(&device, ...);
//     MipDispatchState_registerPacketCallback(&device, &handlers[0], 0x80, 0x04, &myCallback, NULL);
//     MipDispatchState_registerPacketCallback(&device, &gyroHandler,  0x80, 0x05, &myCallback, NULL);
//
//     MipDispatchHandler_init(&handler[0], 0x80, 0x04, &myCallback, NULL);
//     MipInterface_registerHandler(&device, &handler[0]);
// }




#ifdef __cplusplus
} // namespace C
} // namespace mscl
#endif
