
#include "commands_gnss.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_ReceiverInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo* self)
{
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo* self)
{
    return offset;
}


size_t insert_MipCmd_Gnss_ReceiverInfo_Receiverinfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_u8(buffer, bufferSize, offset, self->mip_data_descriptor_set);
    
    assert(32 <= 32);
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->description[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverInfo_Receiverinfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo_Receiverinfo* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->mip_data_descriptor_set);
    
    assert(32 <= 32);
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->description[i]);
    
    return offset;
}


size_t insert_MipCmd_Gnss_ReceiverInfo_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverInfo_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->num_receivers);
    
    assert(self->num_receivers <= 5);
    for(unsigned int i=0; i < self->num_receivers; i++)
        offset = insert_MipCmd_Gnss_ReceiverInfo_Receiverinfo(buffer, bufferSize, offset, &self->receiver_info[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverInfo_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverInfo_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->num_receivers);
    
    assert(self->num_receivers <= 5);
    for(unsigned int i=0; i < self->num_receivers; i++)
        offset = extract_MipCmd_Gnss_ReceiverInfo_Receiverinfo(buffer, bufferSize, offset, &self->receiver_info[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_SignalConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_SignalConfiguration* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->gps_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->glonass_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->galileo_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->beidou_enable);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_SignalConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_SignalConfiguration* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->gps_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->glonass_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->galileo_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->beidou_enable);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


size_t insert_MipCmd_Gnss_SignalConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_SignalConfiguration_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->gps_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->glonass_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->galileo_enable);
    offset = insert_u8(buffer, bufferSize, offset, self->beidou_enable);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_SignalConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_SignalConfiguration_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->gps_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->glonass_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->galileo_enable);
    offset = extract_u8(buffer, bufferSize, offset, &self->beidou_enable);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_RtkDongleConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_RtkDongleConfiguration* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_RtkDongleConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_RtkDongleConfiguration* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


size_t insert_MipCmd_Gnss_RtkDongleConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_RtkDongleConfiguration_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipCmd_Gnss_RtkDongleConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_RtkDongleConfiguration_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Gnss_ReceiverSafeMode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Gnss_ReceiverSafeMode* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Gnss_ReceiverSafeMode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Gnss_ReceiverSafeMode* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
