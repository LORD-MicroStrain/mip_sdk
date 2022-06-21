
#include "commands_rtk.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

size_t insert_MipConnectedDeviceType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipConnectedDeviceType self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipConnectedDeviceType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipConnectedDeviceType* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipMediaSelector(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipMediaSelector self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipMediaSelector(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipMediaSelector* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipLedAction(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipLedAction self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipLedAction(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipLedAction* self)
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
size_t insert_MipCmd_Rtk_GetStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetStatusFlags* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetStatusFlags* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetStatusFlags_Statusflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Rtk_GetStatusFlags_Statusflags self)
{
    return insert_u32(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_Rtk_GetStatusFlags_Statusflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Rtk_GetStatusFlags_Statusflags* self)
{
    uint32_t tmp;
    offset = extract_u32(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_Rtk_GetStatusFlags_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetStatusFlags_Response* self)
{
    offset = insert_MipCmd_Rtk_GetStatusFlags_Statusflags(buffer, bufferSize, offset, self->flags);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetStatusFlags_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetStatusFlags_Response* self)
{
    offset = extract_MipCmd_Rtk_GetStatusFlags_Statusflags(buffer, bufferSize, offset, &self->flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetImei(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImei* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImei(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImei* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetImei_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImei_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->IMEI[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImei_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImei_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->IMEI[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetImsi(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImsi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImsi(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImsi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetImsi_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetImsi_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->IMSI[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetImsi_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetImsi_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->IMSI[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetIccid(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetIccid* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetIccid(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetIccid* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetIccid_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetIccid_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ICCID[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetIccid_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetIccid_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ICCID[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_ConnectedDeviceType(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ConnectedDeviceType* self)
{
    offset = insert_MipConnectedDeviceType(buffer, bufferSize, offset, self->devType);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ConnectedDeviceType(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ConnectedDeviceType* self)
{
    offset = extract_MipConnectedDeviceType(buffer, bufferSize, offset, &self->devType);
    
    return offset;
}


size_t insert_MipCmd_Rtk_ConnectedDeviceType_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ConnectedDeviceType_Response* self)
{
    offset = insert_MipConnectedDeviceType(buffer, bufferSize, offset, self->devType);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ConnectedDeviceType_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ConnectedDeviceType_Response* self)
{
    offset = extract_MipConnectedDeviceType(buffer, bufferSize, offset, &self->devType);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetActCode(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetActCode* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetActCode(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetActCode* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetActCode_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetActCode_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ActivationCode[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetActCode_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetActCode_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ActivationCode[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetModemFirmwareVersion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetModemFirmwareVersion* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetModemFirmwareVersion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetModemFirmwareVersion* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetModemFirmwareVersion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetModemFirmwareVersion_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = insert_char(buffer, bufferSize, offset, self->ModemFirmwareVersion[i]);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetModemFirmwareVersion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetModemFirmwareVersion_Response* self)
{
    for(unsigned int i=0; i < 32; i++)
        offset = extract_char(buffer, bufferSize, offset, &self->ModemFirmwareVersion[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_GetRssi(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetRssi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetRssi(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetRssi* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_GetRssi_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_GetRssi_Response* self)
{
    offset = insert_bool(buffer, bufferSize, offset, self->valid);
    offset = insert_s32(buffer, bufferSize, offset, self->rssi);
    offset = insert_s32(buffer, bufferSize, offset, self->signalQuality);
    
    return offset;
}

size_t extract_MipCmd_Rtk_GetRssi_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_GetRssi_Response* self)
{
    offset = extract_bool(buffer, bufferSize, offset, &self->valid);
    offset = extract_s32(buffer, bufferSize, offset, &self->rssi);
    offset = extract_s32(buffer, bufferSize, offset, &self->signalQuality);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_ServiceStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ServiceStatus* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->reserved1);
    offset = insert_u32(buffer, bufferSize, offset, self->reserved2);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ServiceStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ServiceStatus* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->reserved1);
    offset = extract_u32(buffer, bufferSize, offset, &self->reserved2);
    
    return offset;
}


size_t insert_MipCmd_Rtk_ServiceStatus_Serviceflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipCmd_Rtk_ServiceStatus_Serviceflags self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipCmd_Rtk_ServiceStatus_Serviceflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipCmd_Rtk_ServiceStatus_Serviceflags* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipCmd_Rtk_ServiceStatus_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ServiceStatus_Response* self)
{
    offset = insert_MipCmd_Rtk_ServiceStatus_Serviceflags(buffer, bufferSize, offset, self->flags);
    offset = insert_u32(buffer, bufferSize, offset, self->recievedBytes);
    offset = insert_u32(buffer, bufferSize, offset, self->lastBytes);
    offset = insert_u64(buffer, bufferSize, offset, self->lastBytesTime);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ServiceStatus_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ServiceStatus_Response* self)
{
    offset = extract_MipCmd_Rtk_ServiceStatus_Serviceflags(buffer, bufferSize, offset, &self->flags);
    offset = extract_u32(buffer, bufferSize, offset, &self->recievedBytes);
    offset = extract_u32(buffer, bufferSize, offset, &self->lastBytes);
    offset = extract_u64(buffer, bufferSize, offset, &self->lastBytesTime);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_ProdEraseStorage(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ProdEraseStorage* self)
{
    offset = insert_MipMediaSelector(buffer, bufferSize, offset, self->media);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ProdEraseStorage(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ProdEraseStorage* self)
{
    offset = extract_MipMediaSelector(buffer, bufferSize, offset, &self->media);
    
    return offset;
}


size_t insert_MipCmd_Rtk_ProdEraseStorage_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ProdEraseStorage_Response* self)
{
    offset = insert_MipMediaSelector(buffer, bufferSize, offset, self->media);
    
    return offset;
}

size_t extract_MipCmd_Rtk_ProdEraseStorage_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ProdEraseStorage_Response* self)
{
    offset = extract_MipMediaSelector(buffer, bufferSize, offset, &self->media);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_LedControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_LedControl* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->altColor[i]);
    offset = insert_MipLedAction(buffer, bufferSize, offset, self->act);
    offset = insert_u32(buffer, bufferSize, offset, self->period);
    
    return offset;
}

size_t extract_MipCmd_Rtk_LedControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_LedControl* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->altColor[i]);
    offset = extract_MipLedAction(buffer, bufferSize, offset, &self->act);
    offset = extract_u32(buffer, bufferSize, offset, &self->period);
    
    return offset;
}


size_t insert_MipCmd_Rtk_LedControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_LedControl_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->altColor[i]);
    offset = insert_MipLedAction(buffer, bufferSize, offset, self->act);
    offset = insert_u32(buffer, bufferSize, offset, self->period);
    
    return offset;
}

size_t extract_MipCmd_Rtk_LedControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_LedControl_Response* self)
{
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->primaryColor[i]);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->altColor[i]);
    offset = extract_MipLedAction(buffer, bufferSize, offset, &self->act);
    offset = extract_u32(buffer, bufferSize, offset, &self->period);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Rtk_ModemHardReset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ModemHardReset* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_ModemHardReset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ModemHardReset* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}


size_t insert_MipCmd_Rtk_ModemHardReset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Rtk_ModemHardReset_Response* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}

size_t extract_MipCmd_Rtk_ModemHardReset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Rtk_ModemHardReset_Response* self)
{
    (void)buffer;
    (void)bufferSize;
    (void)self;
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
