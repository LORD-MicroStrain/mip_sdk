
#include "commands_3dm.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

size_t insert_MipPpsSource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipPpsSource self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipPpsSource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipPpsSource* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipSensorRangeType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipSensorRangeType self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipSensorRangeType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipSensorRangeType* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollImuMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollImuMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollImuMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollImuMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollGnssMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollGnssMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollGnssMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollGnssMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollFilterMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollFilterMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollFilterMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollFilterMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollNmeaMessage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollNmeaMessage* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollNmeaMessage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollNmeaMessage* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_ImuMessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuMessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuMessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuMessageFormat* self)
{
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_ImuMessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuMessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuMessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuMessageFormat_Response* self)
{
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpsMessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsMessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsMessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsMessageFormat* self)
{
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_GpsMessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsMessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsMessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsMessageFormat_Response* self)
{
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_FilterMessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterMessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterMessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterMessageFormat* self)
{
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_FilterMessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterMessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterMessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterMessageFormat_Response* self)
{
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_ImuGetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuGetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_3dm_ImuGetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ImuGetBaseRate_Response* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_ImuGetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ImuGetBaseRate_Response* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpsGetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsGetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_3dm_GpsGetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpsGetBaseRate_Response* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpsGetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpsGetBaseRate_Response* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_FilterGetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterGetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterGetBaseRate* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_3dm_FilterGetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FilterGetBaseRate_Response* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_FilterGetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FilterGetBaseRate_Response* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PollData(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PollData* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_bool(buffer, bufferSize, offset, self->suppress_ack);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_PollData(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PollData* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_bool(buffer, bufferSize, offset, &self->suppress_ack);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GetBaseRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GetBaseRate* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    
    return offset;
}

size_t extract_MipCmd_3dm_GetBaseRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GetBaseRate* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    
    return offset;
}


size_t insert_MipCmd_3dm_GetBaseRate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GetBaseRate_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u16(buffer, bufferSize, offset, self->rate);
    
    return offset;
}

size_t extract_MipCmd_3dm_GetBaseRate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GetBaseRate_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_u16(buffer, bufferSize, offset, &self->rate);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_MessageFormat(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MessageFormat* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MessageFormat(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MessageFormat* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_MessageFormat_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MessageFormat_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u8(buffer, bufferSize, offset, self->num_descriptors);
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = insert_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MessageFormat_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MessageFormat_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    uint8_t num_descriptors;
    offset = extract_u8(buffer, bufferSize, offset, &num_descriptors);
    if( num_descriptors < self->num_descriptors )
        self->num_descriptors = num_descriptors;
    for(unsigned int i=0; i < self->num_descriptors; i++)
        offset = extract_MipDescriptorRate(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_DeviceSettings(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_DeviceSettings* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_3dm_DeviceSettings(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_DeviceSettings* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_UartBaudrate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_UartBaudrate* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_3dm_UartBaudrate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_UartBaudrate* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


size_t insert_MipCmd_3dm_UartBaudrate_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_UartBaudrate_Response* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->baud);
    
    return offset;
}

size_t extract_MipCmd_3dm_UartBaudrate_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_UartBaudrate_Response* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->baud);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_FactoryStreaming_Action(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_FactoryStreaming_Action self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_FactoryStreaming_Action(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_FactoryStreaming_Action* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_FactoryStreaming(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_FactoryStreaming* self)
{
    offset = insert_MipCmd_3dm_FactoryStreaming_Action(buffer, bufferSize, offset, self->action);
    offset = insert_u8(buffer, bufferSize, offset, self->reserved);
    
    return offset;
}

size_t extract_MipCmd_3dm_FactoryStreaming(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_FactoryStreaming* self)
{
    offset = extract_MipCmd_3dm_FactoryStreaming_Action(buffer, bufferSize, offset, &self->action);
    offset = extract_u8(buffer, bufferSize, offset, &self->reserved);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_DatastreamControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_DatastreamControl* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_3dm_DatastreamControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_DatastreamControl* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_MipCmd_3dm_DatastreamControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_DatastreamControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_bool(buffer, bufferSize, offset, self->enabled);
    
    return offset;
}

size_t extract_MipCmd_3dm_DatastreamControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_DatastreamControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_bool(buffer, bufferSize, offset, &self->enabled);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GnssSbasSettings_Sbasoptions self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GnssSbasSettings_Sbasoptions* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_3dm_GnssSbasSettings(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssSbasSettings* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable_sbas);
    offset = insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, self->sbas_options);
    offset = insert_u8(buffer, bufferSize, offset, self->num_included_prns);
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->included_prns[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssSbasSettings(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssSbasSettings* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable_sbas);
    offset = extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, &self->sbas_options);
    uint8_t num_included_prns;
    offset = extract_u8(buffer, bufferSize, offset, &num_included_prns);
    if( num_included_prns < self->num_included_prns )
        self->num_included_prns = num_included_prns;
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->included_prns[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_GnssSbasSettings_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssSbasSettings_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable_sbas);
    offset = insert_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, self->sbas_options);
    offset = insert_u8(buffer, bufferSize, offset, self->num_included_prns);
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->included_prns[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssSbasSettings_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssSbasSettings_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable_sbas);
    offset = extract_MipCmd_3dm_GnssSbasSettings_Sbasoptions(buffer, bufferSize, offset, &self->sbas_options);
    uint8_t num_included_prns;
    offset = extract_u8(buffer, bufferSize, offset, &num_included_prns);
    if( num_included_prns < self->num_included_prns )
        self->num_included_prns = num_included_prns;
    for(unsigned int i=0; i < self->num_included_prns; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->included_prns[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GnssTimeAssistance(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssTimeAssistance* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_float(buffer, bufferSize, offset, self->accuracy);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssTimeAssistance(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssTimeAssistance* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_float(buffer, bufferSize, offset, &self->accuracy);
    
    return offset;
}


size_t insert_MipCmd_3dm_GnssTimeAssistance_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GnssTimeAssistance_Response* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_float(buffer, bufferSize, offset, self->accuracy);
    
    return offset;
}

size_t extract_MipCmd_3dm_GnssTimeAssistance_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GnssTimeAssistance_Response* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_float(buffer, bufferSize, offset, &self->accuracy);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_AdvLowpassFilter(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AdvLowpassFilter* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->target_descriptor);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    offset = insert_bool(buffer, bufferSize, offset, self->manual);
    offset = insert_u16(buffer, bufferSize, offset, self->frequency);
    offset = insert_u8(buffer, bufferSize, offset, self->reserved);
    
    return offset;
}

size_t extract_MipCmd_3dm_AdvLowpassFilter(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AdvLowpassFilter* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->target_descriptor);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->manual);
    offset = extract_u16(buffer, bufferSize, offset, &self->frequency);
    offset = extract_u8(buffer, bufferSize, offset, &self->reserved);
    
    return offset;
}


size_t insert_MipCmd_3dm_AdvLowpassFilter_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AdvLowpassFilter_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->target_descriptor);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    offset = insert_bool(buffer, bufferSize, offset, self->manual);
    offset = insert_u16(buffer, bufferSize, offset, self->frequency);
    offset = insert_u8(buffer, bufferSize, offset, self->reserved);
    
    return offset;
}

size_t extract_MipCmd_3dm_AdvLowpassFilter_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AdvLowpassFilter_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->target_descriptor);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->manual);
    offset = extract_u16(buffer, bufferSize, offset, &self->frequency);
    offset = extract_u8(buffer, bufferSize, offset, &self->reserved);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_PpsSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PpsSource* self)
{
    offset = insert_MipPpsSource(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_3dm_PpsSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PpsSource* self)
{
    offset = extract_MipPpsSource(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


size_t insert_MipCmd_3dm_PpsSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_PpsSource_Response* self)
{
    offset = insert_MipPpsSource(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_3dm_PpsSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_PpsSource_Response* self)
{
    offset = extract_MipPpsSource(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpioConfig_Feature(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GpioConfig_Feature self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GpioConfig_Feature(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GpioConfig_Feature* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_GpioConfig_Behavior(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GpioConfig_Behavior self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GpioConfig_Behavior(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GpioConfig_Behavior* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_GpioConfig_Pinmode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_GpioConfig_Pinmode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_GpioConfig_Pinmode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_GpioConfig_Pinmode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_3dm_GpioConfig(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioConfig* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, self->feature);
    offset = insert_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, self->behavior);
    offset = insert_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, self->pin_mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioConfig(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioConfig* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, &self->feature);
    offset = extract_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, &self->behavior);
    offset = extract_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, &self->pin_mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_GpioConfig_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioConfig_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, self->feature);
    offset = insert_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, self->behavior);
    offset = insert_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, self->pin_mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioConfig_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioConfig_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_GpioConfig_Feature(buffer, bufferSize, offset, &self->feature);
    offset = extract_MipCmd_3dm_GpioConfig_Behavior(buffer, bufferSize, offset, &self->behavior);
    offset = extract_MipCmd_3dm_GpioConfig_Pinmode(buffer, bufferSize, offset, &self->pin_mode);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GpioState(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioState* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_bool(buffer, bufferSize, offset, self->state);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioState(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioState* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_bool(buffer, bufferSize, offset, &self->state);
    
    return offset;
}


size_t insert_MipCmd_3dm_GpioState_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GpioState_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_bool(buffer, bufferSize, offset, self->state);
    
    return offset;
}

size_t extract_MipCmd_3dm_GpioState_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GpioState_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_bool(buffer, bufferSize, offset, &self->state);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Odometer_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_Odometer_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_Odometer_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_Odometer_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_Odometer(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Odometer* self)
{
    offset = insert_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, self->mode);
    offset = insert_float(buffer, bufferSize, offset, self->scaling);
    offset = insert_float(buffer, bufferSize, offset, self->uncertainty);
    
    return offset;
}

size_t extract_MipCmd_3dm_Odometer(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Odometer* self)
{
    offset = extract_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, &self->mode);
    offset = extract_float(buffer, bufferSize, offset, &self->scaling);
    offset = extract_float(buffer, bufferSize, offset, &self->uncertainty);
    
    return offset;
}


size_t insert_MipCmd_3dm_Odometer_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Odometer_Response* self)
{
    offset = insert_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, self->mode);
    offset = insert_float(buffer, bufferSize, offset, self->scaling);
    offset = insert_float(buffer, bufferSize, offset, self->uncertainty);
    
    return offset;
}

size_t extract_MipCmd_3dm_Odometer_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Odometer_Response* self)
{
    offset = extract_MipCmd_3dm_Odometer_Mode(buffer, bufferSize, offset, &self->mode);
    offset = extract_float(buffer, bufferSize, offset, &self->scaling);
    offset = extract_float(buffer, bufferSize, offset, &self->uncertainty);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventSupport_Query(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventSupport_Query self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventSupport_Query(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventSupport_Query* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventSupport(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventSupport* self)
{
    offset = insert_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, self->query);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventSupport(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventSupport* self)
{
    offset = extract_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, &self->query);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventSupport_Info(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventSupport_Info* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventSupport_Info(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventSupport_Info* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventSupport_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventSupport_Response* self)
{
    offset = insert_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, self->query);
    offset = insert_u8(buffer, bufferSize, offset, self->max_instances);
    offset = insert_u8(buffer, bufferSize, offset, self->num_entries);
    assert(self->num_entries <= 126);
    for(unsigned int i=0; i < self->num_entries; i++)
        offset = insert_MipCmd_3dm_EventSupport_Info(buffer, bufferSize, offset, &self->entries[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventSupport_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventSupport_Response* self)
{
    offset = extract_MipCmd_3dm_EventSupport_Query(buffer, bufferSize, offset, &self->query);
    offset = extract_u8(buffer, bufferSize, offset, &self->max_instances);
    uint8_t num_entries;
    offset = extract_u8(buffer, bufferSize, offset, &num_entries);
    if( num_entries < self->num_entries )
        self->num_entries = num_entries;
    assert(self->num_entries <= 126);
    for(unsigned int i=0; i < self->num_entries; i++)
        offset = extract_MipCmd_3dm_EventSupport_Info(buffer, bufferSize, offset, &self->entries[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventControl_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventControl_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventControl_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventControl_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventControl* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventControl* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventControl_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventTriggerStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTriggerStatus* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->requested_instances[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTriggerStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTriggerStatus* self)
{
    uint8_t requested_count;
    offset = extract_u8(buffer, bufferSize, offset, &requested_count);
    if( requested_count < self->requested_count )
        self->requested_count = requested_count;
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->requested_instances[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTriggerStatus_Status(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTriggerStatus_Status self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTriggerStatus_Status(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTriggerStatus_Status* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_3dm_EventTriggerStatus_Entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTriggerStatus_Entry* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    offset = insert_MipCmd_3dm_EventTriggerStatus_Status(buffer, bufferSize, offset, self->status);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTriggerStatus_Entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTriggerStatus_Entry* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    offset = extract_MipCmd_3dm_EventTriggerStatus_Status(buffer, bufferSize, offset, &self->status);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTriggerStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTriggerStatus_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    for(unsigned int i=0; i < self->count; i++)
        offset = insert_MipCmd_3dm_EventTriggerStatus_Entry(buffer, bufferSize, offset, &self->triggers[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTriggerStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTriggerStatus_Response* self)
{
    uint8_t count;
    offset = extract_u8(buffer, bufferSize, offset, &count);
    if( count < self->count )
        self->count = count;
    for(unsigned int i=0; i < self->count; i++)
        offset = extract_MipCmd_3dm_EventTriggerStatus_Entry(buffer, bufferSize, offset, &self->triggers[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventActionStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventActionStatus* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->requested_count);
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->requested_instances[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventActionStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventActionStatus* self)
{
    uint8_t requested_count;
    offset = extract_u8(buffer, bufferSize, offset, &requested_count);
    if( requested_count < self->requested_count )
        self->requested_count = requested_count;
    for(unsigned int i=0; i < self->requested_count; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->requested_instances[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventActionStatus_Entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventActionStatus_Entry* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->action_type);
    offset = insert_u8(buffer, bufferSize, offset, self->trigger_id);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventActionStatus_Entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventActionStatus_Entry* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->action_type);
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger_id);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventActionStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventActionStatus_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    for(unsigned int i=0; i < self->count; i++)
        offset = insert_MipCmd_3dm_EventActionStatus_Entry(buffer, bufferSize, offset, &self->actions[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventActionStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventActionStatus_Response* self)
{
    uint8_t count;
    offset = extract_u8(buffer, bufferSize, offset, &count);
    if( count < self->count )
        self->count = count;
    for(unsigned int i=0; i < self->count; i++)
        offset = extract_MipCmd_3dm_EventActionStatus_Entry(buffer, bufferSize, offset, &self->actions[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventTrigger_Type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTrigger_Type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTrigger_Type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTrigger_Type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventTrigger_Gpioparams_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTrigger_Gpioparams_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTrigger_Gpioparams_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTrigger_Gpioparams_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventTrigger_Gpioparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Gpioparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_EventTrigger_Gpioparams_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Gpioparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Gpioparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_EventTrigger_Gpioparams_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger_Thresholdparams_Type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventTrigger_Thresholdparams_Type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventTrigger_Thresholdparams_Type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventTrigger_Thresholdparams_Type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventTrigger_Thresholdparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Thresholdparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u8(buffer, bufferSize, offset, self->field_desc);
    offset = insert_u8(buffer, bufferSize, offset, self->param_id);
    offset = insert_MipCmd_3dm_EventTrigger_Thresholdparams_Type(buffer, bufferSize, offset, self->type);
    offset = insert_double(buffer, bufferSize, offset, self->low_thres);
    offset = insert_double(buffer, bufferSize, offset, self->int_thres);
    offset = insert_double(buffer, bufferSize, offset, self->high_thres);
    offset = insert_double(buffer, bufferSize, offset, self->interval);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Thresholdparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Thresholdparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_u8(buffer, bufferSize, offset, &self->field_desc);
    offset = extract_u8(buffer, bufferSize, offset, &self->param_id);
    offset = extract_MipCmd_3dm_EventTrigger_Thresholdparams_Type(buffer, bufferSize, offset, &self->type);
    offset = extract_double(buffer, bufferSize, offset, &self->low_thres);
    offset = extract_double(buffer, bufferSize, offset, &self->int_thres);
    offset = extract_double(buffer, bufferSize, offset, &self->high_thres);
    offset = extract_double(buffer, bufferSize, offset, &self->interval);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger_Combinationparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Combinationparams* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->logic_table);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->input_triggers[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Combinationparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Combinationparams* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->logic_table);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->input_triggers[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, self->type);
    offset = insert_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = insert_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    offset = insert_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, &self->type);
    offset = extract_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = extract_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    offset = extract_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventTrigger_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventTrigger_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, self->type);
    offset = insert_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = insert_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    offset = insert_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventTrigger_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventTrigger_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_MipCmd_3dm_EventTrigger_Type(buffer, bufferSize, offset, &self->type);
    offset = extract_MipCmd_3dm_EventTrigger_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = extract_MipCmd_3dm_EventTrigger_Thresholdparams(buffer, bufferSize, offset, &self->threshold);
    offset = extract_MipCmd_3dm_EventTrigger_Combinationparams(buffer, bufferSize, offset, &self->combination);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_EventAction_Type(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventAction_Type self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventAction_Type(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventAction_Type* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventAction_Gpioparams_Mode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_3dm_EventAction_Gpioparams_Mode self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_3dm_EventAction_Gpioparams_Mode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_3dm_EventAction_Gpioparams_Mode* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipCmd_3dm_EventAction_Gpioparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction_Gpioparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->pin);
    offset = insert_MipCmd_3dm_EventAction_Gpioparams_Mode(buffer, bufferSize, offset, self->mode);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction_Gpioparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction_Gpioparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->pin);
    offset = extract_MipCmd_3dm_EventAction_Gpioparams_Mode(buffer, bufferSize, offset, &self->mode);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventAction_Messageparams(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction_Messageparams* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->desc_set);
    offset = insert_u16(buffer, bufferSize, offset, self->decimation);
    offset = insert_u8(buffer, bufferSize, offset, self->num_fields);
    for(unsigned int i=0; i < self->num_fields; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->descriptors[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction_Messageparams(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction_Messageparams* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->desc_set);
    offset = extract_u16(buffer, bufferSize, offset, &self->decimation);
    uint8_t num_fields;
    offset = extract_u8(buffer, bufferSize, offset, &num_fields);
    if( num_fields < self->num_fields )
        self->num_fields = num_fields;
    for(unsigned int i=0; i < self->num_fields; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->descriptors[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventAction(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_u8(buffer, bufferSize, offset, self->trigger);
    offset = insert_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, self->type);
    offset = insert_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = insert_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger);
    offset = extract_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, &self->type);
    offset = extract_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = extract_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}


size_t insert_MipCmd_3dm_EventAction_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_EventAction_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->instance);
    offset = insert_u8(buffer, bufferSize, offset, self->trigger);
    offset = insert_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, self->type);
    offset = insert_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = insert_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}

size_t extract_MipCmd_3dm_EventAction_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_EventAction_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->instance);
    offset = extract_u8(buffer, bufferSize, offset, &self->trigger);
    offset = extract_MipCmd_3dm_EventAction_Type(buffer, bufferSize, offset, &self->type);
    offset = extract_MipCmd_3dm_EventAction_Gpioparams(buffer, bufferSize, offset, &self->gpio);
    offset = extract_MipCmd_3dm_EventAction_Messageparams(buffer, bufferSize, offset, &self->message);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_AccelBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AccelBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_AccelBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AccelBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_AccelBias_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_AccelBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_AccelBias_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_AccelBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_GyroBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GyroBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GyroBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GyroBias* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_GyroBias_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_GyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_GyroBias_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_GyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_CaptureGyroBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CaptureGyroBias* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->averaging_time_ms);
    
    return offset;
}

size_t extract_MipCmd_3dm_CaptureGyroBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CaptureGyroBias* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->averaging_time_ms);
    
    return offset;
}


size_t insert_MipCmd_3dm_CaptureGyroBias_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CaptureGyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_CaptureGyroBias_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CaptureGyroBias_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_MagHardIronOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagHardIronOffset* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagHardIronOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagHardIronOffset* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_MagHardIronOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagHardIronOffset_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagHardIronOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagHardIronOffset_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_MagSoftIronMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagSoftIronMatrix* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagSoftIronMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagSoftIronMatrix* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_MagSoftIronMatrix_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_MagSoftIronMatrix_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_MagSoftIronMatrix_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_MagSoftIronMatrix_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Sensor2VehicleTransformEuler(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformEuler* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformEuler(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformEuler* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


size_t insert_MipCmd_3dm_Sensor2VehicleTransformEuler_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformEuler_Response* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformEuler_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformEuler_Response* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Sensor2VehicleTransformQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformQuaternion* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformQuaternion* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformQuaternion_Response* self)
{
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_Sensor2VehicleTransformDcm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformDcm* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformDcm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformDcm* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


size_t insert_MipCmd_3dm_Sensor2VehicleTransformDcm_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_Sensor2VehicleTransformDcm_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_Sensor2VehicleTransformDcm_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_Sensor2VehicleTransformDcm_Response* self)
{
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_ComplementaryFilter(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ComplementaryFilter* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->pitch_roll_enable);
    offset = insert_bool(buffer, bufferSize, offset, self->heading_enable);
    offset = insert_float(buffer, bufferSize, offset, self->pitch_roll_time_constant);
    offset = insert_float(buffer, bufferSize, offset, self->heading_time_constant);
    
    return offset;
}

size_t extract_MipCmd_3dm_ComplementaryFilter(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ComplementaryFilter* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->pitch_roll_enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->heading_enable);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch_roll_time_constant);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_time_constant);
    
    return offset;
}


size_t insert_MipCmd_3dm_ComplementaryFilter_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_ComplementaryFilter_Response* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->pitch_roll_enable);
    offset = insert_bool(buffer, bufferSize, offset, self->heading_enable);
    offset = insert_float(buffer, bufferSize, offset, self->pitch_roll_time_constant);
    offset = insert_float(buffer, bufferSize, offset, self->heading_time_constant);
    
    return offset;
}

size_t extract_MipCmd_3dm_ComplementaryFilter_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_ComplementaryFilter_Response* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->pitch_roll_enable);
    offset = extract_bool(buffer, bufferSize, offset, &self->heading_enable);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch_roll_time_constant);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_time_constant);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_SensorRange(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_SensorRange* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    offset = insert_u8(buffer, bufferSize, offset, self->setting);
    
    return offset;
}

size_t extract_MipCmd_3dm_SensorRange(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_SensorRange* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    offset = extract_u8(buffer, bufferSize, offset, &self->setting);
    
    return offset;
}


size_t insert_MipCmd_3dm_SensorRange_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_SensorRange_Response* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    offset = insert_u8(buffer, bufferSize, offset, self->setting);
    
    return offset;
}

size_t extract_MipCmd_3dm_SensorRange_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_SensorRange_Response* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    offset = extract_u8(buffer, bufferSize, offset, &self->setting);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_3dm_CalibratedSensorRanges(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CalibratedSensorRanges* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    
    return offset;
}

size_t extract_MipCmd_3dm_CalibratedSensorRanges(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CalibratedSensorRanges* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    
    return offset;
}


size_t insert_MipCmd_3dm_CalibratedSensorRanges_Entry(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CalibratedSensorRanges_Entry* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->setting);
    offset = insert_float(buffer, bufferSize, offset, self->range);
    
    return offset;
}

size_t extract_MipCmd_3dm_CalibratedSensorRanges_Entry(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CalibratedSensorRanges_Entry* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->setting);
    offset = extract_float(buffer, bufferSize, offset, &self->range);
    
    return offset;
}


size_t insert_MipCmd_3dm_CalibratedSensorRanges_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_3dm_CalibratedSensorRanges_Response* self)
{
    offset = insert_MipSensorRangeType(buffer, bufferSize, offset, self->sensor);
    offset = insert_u8(buffer, bufferSize, offset, self->num_ranges);
    assert(self->num_ranges <= 50);
    for(unsigned int i=0; i < self->num_ranges; i++)
        offset = insert_MipCmd_3dm_CalibratedSensorRanges_Entry(buffer, bufferSize, offset, &self->ranges[i]);
    
    return offset;
}

size_t extract_MipCmd_3dm_CalibratedSensorRanges_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_3dm_CalibratedSensorRanges_Response* self)
{
    offset = extract_MipSensorRangeType(buffer, bufferSize, offset, &self->sensor);
    uint8_t num_ranges;
    offset = extract_u8(buffer, bufferSize, offset, &num_ranges);
    if( num_ranges < self->num_ranges )
        self->num_ranges = num_ranges;
    assert(self->num_ranges <= 50);
    for(unsigned int i=0; i < self->num_ranges; i++)
        offset = extract_MipCmd_3dm_CalibratedSensorRanges_Entry(buffer, bufferSize, offset, &self->ranges[i]);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
