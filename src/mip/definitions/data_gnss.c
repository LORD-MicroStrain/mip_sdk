
#include "data_gnss.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

size_t insert_MipGnssConstellationId(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipGnssConstellationId self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipGnssConstellationId(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipGnssConstellationId* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipGnssSignalId(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipGnssSignalId self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipGnssSignalId(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipGnssSignalId* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipSbasSystem(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipSbasSystem self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipSbasSystem(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipSbasSystem* self)
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
size_t insert_MipData_Gnss_LlhPos_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_LlhPos_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_LlhPos_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_LlhPos_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_LlhPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_LlhPos* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->latitude);
    offset = insert_double(buffer, bufferSize, offset, self->longitude);
    offset = insert_double(buffer, bufferSize, offset, self->ellipsoid_height);
    offset = insert_double(buffer, bufferSize, offset, self->msl_height);
    offset = insert_float(buffer, bufferSize, offset, self->horizontal_accuracy);
    offset = insert_float(buffer, bufferSize, offset, self->vertical_accuracy);
    offset = insert_MipData_Gnss_LlhPos_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_LlhPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_LlhPos* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->latitude);
    offset = extract_double(buffer, bufferSize, offset, &self->longitude);
    offset = extract_double(buffer, bufferSize, offset, &self->ellipsoid_height);
    offset = extract_double(buffer, bufferSize, offset, &self->msl_height);
    offset = extract_float(buffer, bufferSize, offset, &self->horizontal_accuracy);
    offset = extract_float(buffer, bufferSize, offset, &self->vertical_accuracy);
    offset = extract_MipData_Gnss_LlhPos_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_EcefPos_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_EcefPos_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_EcefPos_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_EcefPos_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_EcefPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_EcefPos* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->x[i]);
    offset = insert_float(buffer, bufferSize, offset, self->x_accuracy);
    offset = insert_MipData_Gnss_EcefPos_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_EcefPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_EcefPos* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->x[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->x_accuracy);
    offset = extract_MipData_Gnss_EcefPos_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_NedVel_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_NedVel_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_NedVel_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_NedVel_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_NedVel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_NedVel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->v[i]);
    offset = insert_float(buffer, bufferSize, offset, self->speed);
    offset = insert_float(buffer, bufferSize, offset, self->ground_speed);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->speed_accuracy);
    offset = insert_float(buffer, bufferSize, offset, self->heading_accuracy);
    offset = insert_MipData_Gnss_NedVel_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_NedVel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_NedVel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->v[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->speed);
    offset = extract_float(buffer, bufferSize, offset, &self->ground_speed);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->speed_accuracy);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_accuracy);
    offset = extract_MipData_Gnss_NedVel_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_EcefVel_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_EcefVel_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_EcefVel_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_EcefVel_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_EcefVel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_EcefVel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->v[i]);
    offset = insert_float(buffer, bufferSize, offset, self->v_accuracy);
    offset = insert_MipData_Gnss_EcefVel_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_EcefVel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_EcefVel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->v[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->v_accuracy);
    offset = extract_MipData_Gnss_EcefVel_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_Dop_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_Dop_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_Dop_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_Dop_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_Dop(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_Dop* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->gdop);
    offset = insert_float(buffer, bufferSize, offset, self->pdop);
    offset = insert_float(buffer, bufferSize, offset, self->hdop);
    offset = insert_float(buffer, bufferSize, offset, self->vdop);
    offset = insert_float(buffer, bufferSize, offset, self->tdop);
    offset = insert_float(buffer, bufferSize, offset, self->ndop);
    offset = insert_float(buffer, bufferSize, offset, self->edop);
    offset = insert_MipData_Gnss_Dop_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_Dop(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_Dop* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->gdop);
    offset = extract_float(buffer, bufferSize, offset, &self->pdop);
    offset = extract_float(buffer, bufferSize, offset, &self->hdop);
    offset = extract_float(buffer, bufferSize, offset, &self->vdop);
    offset = extract_float(buffer, bufferSize, offset, &self->tdop);
    offset = extract_float(buffer, bufferSize, offset, &self->ndop);
    offset = extract_float(buffer, bufferSize, offset, &self->edop);
    offset = extract_MipData_Gnss_Dop_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_UtcTime_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_UtcTime_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_UtcTime_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_UtcTime_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_UtcTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_UtcTime* self)
{
    offset = insert_u16(buffer, bufferSize, offset, self->year);
    offset = insert_u8(buffer, bufferSize, offset, self->month);
    offset = insert_u8(buffer, bufferSize, offset, self->day);
    offset = insert_u8(buffer, bufferSize, offset, self->hour);
    offset = insert_u8(buffer, bufferSize, offset, self->min);
    offset = insert_u8(buffer, bufferSize, offset, self->sec);
    offset = insert_u32(buffer, bufferSize, offset, self->msec);
    offset = insert_MipData_Gnss_UtcTime_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_UtcTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_UtcTime* self)
{
    offset = extract_u16(buffer, bufferSize, offset, &self->year);
    offset = extract_u8(buffer, bufferSize, offset, &self->month);
    offset = extract_u8(buffer, bufferSize, offset, &self->day);
    offset = extract_u8(buffer, bufferSize, offset, &self->hour);
    offset = extract_u8(buffer, bufferSize, offset, &self->min);
    offset = extract_u8(buffer, bufferSize, offset, &self->sec);
    offset = extract_u32(buffer, bufferSize, offset, &self->msec);
    offset = extract_MipData_Gnss_UtcTime_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_GpsTime_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsTime_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_GpsTime_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsTime_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_GpsTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsTime* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipData_Gnss_GpsTime_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_GpsTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsTime* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipData_Gnss_GpsTime_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_ClockInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_ClockInfo_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_ClockInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_ClockInfo_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_ClockInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_ClockInfo* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->bias);
    offset = insert_double(buffer, bufferSize, offset, self->drift);
    offset = insert_double(buffer, bufferSize, offset, self->accuracy_estimate);
    offset = insert_MipData_Gnss_ClockInfo_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_ClockInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_ClockInfo* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->bias);
    offset = extract_double(buffer, bufferSize, offset, &self->drift);
    offset = extract_double(buffer, bufferSize, offset, &self->accuracy_estimate);
    offset = extract_MipData_Gnss_ClockInfo_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_FixInfo_Fixtype(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_FixInfo_Fixtype self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_FixInfo_Fixtype(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_FixInfo_Fixtype* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_FixInfo_Fixflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_FixInfo_Fixflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_FixInfo_Fixflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_FixInfo_Fixflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_FixInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_FixInfo_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_FixInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_FixInfo_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_FixInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_FixInfo* self)
{
    offset = insert_MipData_Gnss_FixInfo_Fixtype(buffer, bufferSize, offset, self->fix_type);
    offset = insert_u8(buffer, bufferSize, offset, self->num_sv);
    offset = insert_MipData_Gnss_FixInfo_Fixflags(buffer, bufferSize, offset, self->fix_flags);
    offset = insert_MipData_Gnss_FixInfo_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_FixInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_FixInfo* self)
{
    offset = extract_MipData_Gnss_FixInfo_Fixtype(buffer, bufferSize, offset, &self->fix_type);
    offset = extract_u8(buffer, bufferSize, offset, &self->num_sv);
    offset = extract_MipData_Gnss_FixInfo_Fixflags(buffer, bufferSize, offset, &self->fix_flags);
    offset = extract_MipData_Gnss_FixInfo_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_SvInfo_Svflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SvInfo_Svflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_SvInfo_Svflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SvInfo_Svflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_SvInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SvInfo_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_SvInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SvInfo_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_SvInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SvInfo* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->channel);
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_u16(buffer, bufferSize, offset, self->carrier_noise_ratio);
    offset = insert_s16(buffer, bufferSize, offset, self->azimuth);
    offset = insert_s16(buffer, bufferSize, offset, self->elevation);
    offset = insert_MipData_Gnss_SvInfo_Svflags(buffer, bufferSize, offset, self->sv_flags);
    offset = insert_MipData_Gnss_SvInfo_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_SvInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SvInfo* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->channel);
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_u16(buffer, bufferSize, offset, &self->carrier_noise_ratio);
    offset = extract_s16(buffer, bufferSize, offset, &self->azimuth);
    offset = extract_s16(buffer, bufferSize, offset, &self->elevation);
    offset = extract_MipData_Gnss_SvInfo_Svflags(buffer, bufferSize, offset, &self->sv_flags);
    offset = extract_MipData_Gnss_SvInfo_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_HwStatus_Receiverstate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Receiverstate self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_HwStatus_Receiverstate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Receiverstate* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_HwStatus_Antennastate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Antennastate self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_HwStatus_Antennastate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Antennastate* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_HwStatus_Antennapower(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Antennapower self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_HwStatus_Antennapower(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Antennapower* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_HwStatus_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_HwStatus_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_HwStatus_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_HwStatus_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_HwStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_HwStatus* self)
{
    offset = insert_MipData_Gnss_HwStatus_Receiverstate(buffer, bufferSize, offset, self->receiver_state);
    offset = insert_MipData_Gnss_HwStatus_Antennastate(buffer, bufferSize, offset, self->antenna_state);
    offset = insert_MipData_Gnss_HwStatus_Antennapower(buffer, bufferSize, offset, self->antenna_power);
    offset = insert_MipData_Gnss_HwStatus_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_HwStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_HwStatus* self)
{
    offset = extract_MipData_Gnss_HwStatus_Receiverstate(buffer, bufferSize, offset, &self->receiver_state);
    offset = extract_MipData_Gnss_HwStatus_Antennastate(buffer, bufferSize, offset, &self->antenna_state);
    offset = extract_MipData_Gnss_HwStatus_Antennapower(buffer, bufferSize, offset, &self->antenna_power);
    offset = extract_MipData_Gnss_HwStatus_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_DgpsInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_DgpsInfo_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_DgpsInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_DgpsInfo_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_DgpsInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_DgpsInfo* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_float(buffer, bufferSize, offset, self->age);
    offset = insert_float(buffer, bufferSize, offset, self->range_correction);
    offset = insert_float(buffer, bufferSize, offset, self->range_rate_correction);
    offset = insert_MipData_Gnss_DgpsInfo_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_DgpsInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_DgpsInfo* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_float(buffer, bufferSize, offset, &self->age);
    offset = extract_float(buffer, bufferSize, offset, &self->range_correction);
    offset = extract_float(buffer, bufferSize, offset, &self->range_rate_correction);
    offset = extract_MipData_Gnss_DgpsInfo_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_DgpsChannel_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_DgpsChannel_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_DgpsChannel_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_DgpsChannel_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_DgpsChannel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_DgpsChannel* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_float(buffer, bufferSize, offset, self->age);
    offset = insert_float(buffer, bufferSize, offset, self->range_correction);
    offset = insert_float(buffer, bufferSize, offset, self->range_rate_correction);
    offset = insert_MipData_Gnss_DgpsChannel_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_DgpsChannel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_DgpsChannel* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_float(buffer, bufferSize, offset, &self->age);
    offset = extract_float(buffer, bufferSize, offset, &self->range_correction);
    offset = extract_float(buffer, bufferSize, offset, &self->range_rate_correction);
    offset = extract_MipData_Gnss_DgpsChannel_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_ClockInfo2_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_ClockInfo2_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_ClockInfo2_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_ClockInfo2_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_ClockInfo2(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_ClockInfo2* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->bias);
    offset = insert_double(buffer, bufferSize, offset, self->drift);
    offset = insert_double(buffer, bufferSize, offset, self->bias_accuracy_estimate);
    offset = insert_double(buffer, bufferSize, offset, self->drift_accuracy_estimate);
    offset = insert_MipData_Gnss_ClockInfo2_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_ClockInfo2(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_ClockInfo2* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->bias);
    offset = extract_double(buffer, bufferSize, offset, &self->drift);
    offset = extract_double(buffer, bufferSize, offset, &self->bias_accuracy_estimate);
    offset = extract_double(buffer, bufferSize, offset, &self->drift_accuracy_estimate);
    offset = extract_MipData_Gnss_ClockInfo2_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_GpsLeapSeconds_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsLeapSeconds_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_GpsLeapSeconds_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsLeapSeconds_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_GpsLeapSeconds(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsLeapSeconds* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->leap_seconds);
    offset = insert_MipData_Gnss_GpsLeapSeconds_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_GpsLeapSeconds(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsLeapSeconds* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->leap_seconds);
    offset = extract_MipData_Gnss_GpsLeapSeconds_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_SbasInfo_Sbasstatus(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SbasInfo_Sbasstatus self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_SbasInfo_Sbasstatus(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SbasInfo_Sbasstatus* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_SbasInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SbasInfo_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_SbasInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SbasInfo_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_SbasInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SbasInfo* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipSbasSystem(buffer, bufferSize, offset, self->sbas_system);
    offset = insert_u8(buffer, bufferSize, offset, self->sbas_id);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_MipData_Gnss_SbasInfo_Sbasstatus(buffer, bufferSize, offset, self->sbas_status);
    offset = insert_MipData_Gnss_SbasInfo_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_SbasInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SbasInfo* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipSbasSystem(buffer, bufferSize, offset, &self->sbas_system);
    offset = extract_u8(buffer, bufferSize, offset, &self->sbas_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_MipData_Gnss_SbasInfo_Sbasstatus(buffer, bufferSize, offset, &self->sbas_status);
    offset = extract_MipData_Gnss_SbasInfo_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_SbasCorrection_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SbasCorrection_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_SbasCorrection_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SbasCorrection_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_SbasCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SbasCorrection* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipGnssConstellationId(buffer, bufferSize, offset, self->gnss_id);
    offset = insert_u8(buffer, bufferSize, offset, self->sv_id);
    offset = insert_u8(buffer, bufferSize, offset, self->udrei);
    offset = insert_float(buffer, bufferSize, offset, self->pseudorange_correction);
    offset = insert_float(buffer, bufferSize, offset, self->iono_correction);
    offset = insert_MipData_Gnss_SbasCorrection_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_SbasCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SbasCorrection* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipGnssConstellationId(buffer, bufferSize, offset, &self->gnss_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->sv_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->udrei);
    offset = extract_float(buffer, bufferSize, offset, &self->pseudorange_correction);
    offset = extract_float(buffer, bufferSize, offset, &self->iono_correction);
    offset = extract_MipData_Gnss_SbasCorrection_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_RfErrorDetection_Rfband(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Rfband self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_RfErrorDetection_Rfband(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Rfband* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_RfErrorDetection_Jammingstate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Jammingstate self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_RfErrorDetection_Jammingstate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Jammingstate* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_RfErrorDetection_Spoofingstate(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Spoofingstate self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_RfErrorDetection_Spoofingstate(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Spoofingstate* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_RfErrorDetection_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RfErrorDetection_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_RfErrorDetection_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RfErrorDetection_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_RfErrorDetection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_RfErrorDetection* self)
{
    offset = insert_MipData_Gnss_RfErrorDetection_Rfband(buffer, bufferSize, offset, self->rf_band);
    offset = insert_MipData_Gnss_RfErrorDetection_Jammingstate(buffer, bufferSize, offset, self->jamming_state);
    offset = insert_MipData_Gnss_RfErrorDetection_Spoofingstate(buffer, bufferSize, offset, self->spoofing_state);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    offset = insert_MipData_Gnss_RfErrorDetection_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_RfErrorDetection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_RfErrorDetection* self)
{
    offset = extract_MipData_Gnss_RfErrorDetection_Rfband(buffer, bufferSize, offset, &self->rf_band);
    offset = extract_MipData_Gnss_RfErrorDetection_Jammingstate(buffer, bufferSize, offset, &self->jamming_state);
    offset = extract_MipData_Gnss_RfErrorDetection_Spoofingstate(buffer, bufferSize, offset, &self->spoofing_state);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    offset = extract_MipData_Gnss_RfErrorDetection_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_BaseStationInfo_Indicatorflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_BaseStationInfo_Indicatorflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_BaseStationInfo_Indicatorflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_BaseStationInfo_Indicatorflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_BaseStationInfo_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_BaseStationInfo_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_BaseStationInfo_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_BaseStationInfo_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_BaseStationInfo(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_BaseStationInfo* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->ecef_pos[i]);
    offset = insert_float(buffer, bufferSize, offset, self->height);
    offset = insert_u16(buffer, bufferSize, offset, self->station_id);
    offset = insert_MipData_Gnss_BaseStationInfo_Indicatorflags(buffer, bufferSize, offset, self->indicators);
    offset = insert_MipData_Gnss_BaseStationInfo_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_BaseStationInfo(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_BaseStationInfo* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->ecef_pos[i]);
    offset = extract_float(buffer, bufferSize, offset, &self->height);
    offset = extract_u16(buffer, bufferSize, offset, &self->station_id);
    offset = extract_MipData_Gnss_BaseStationInfo_Indicatorflags(buffer, bufferSize, offset, &self->indicators);
    offset = extract_MipData_Gnss_BaseStationInfo_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_RtkCorrectionsStatus_Epochstatus(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RtkCorrectionsStatus_Epochstatus self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_RtkCorrectionsStatus_Epochstatus(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RtkCorrectionsStatus_Epochstatus* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_RtkCorrectionsStatus_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_RtkCorrectionsStatus_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_RtkCorrectionsStatus_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_RtkCorrectionsStatus_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_RtkCorrectionsStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_RtkCorrectionsStatus* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipData_Gnss_RtkCorrectionsStatus_Epochstatus(buffer, bufferSize, offset, self->epoch_status);
    offset = insert_u32(buffer, bufferSize, offset, self->dongle_status);
    offset = insert_float(buffer, bufferSize, offset, self->gps_correction_latency);
    offset = insert_float(buffer, bufferSize, offset, self->glonass_correction_latency);
    offset = insert_float(buffer, bufferSize, offset, self->galileo_correction_latency);
    offset = insert_float(buffer, bufferSize, offset, self->beidou_correction_latency);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u32(buffer, bufferSize, offset, self->reserved[i]);
    offset = insert_MipData_Gnss_RtkCorrectionsStatus_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_RtkCorrectionsStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_RtkCorrectionsStatus* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipData_Gnss_RtkCorrectionsStatus_Epochstatus(buffer, bufferSize, offset, &self->epoch_status);
    offset = extract_u32(buffer, bufferSize, offset, &self->dongle_status);
    offset = extract_float(buffer, bufferSize, offset, &self->gps_correction_latency);
    offset = extract_float(buffer, bufferSize, offset, &self->glonass_correction_latency);
    offset = extract_float(buffer, bufferSize, offset, &self->galileo_correction_latency);
    offset = extract_float(buffer, bufferSize, offset, &self->beidou_correction_latency);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u32(buffer, bufferSize, offset, &self->reserved[i]);
    offset = extract_MipData_Gnss_RtkCorrectionsStatus_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_SatelliteStatus_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_SatelliteStatus_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_SatelliteStatus_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_SatelliteStatus_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_SatelliteStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_SatelliteStatus* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipGnssConstellationId(buffer, bufferSize, offset, self->gnss_id);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_float(buffer, bufferSize, offset, self->elevation);
    offset = insert_float(buffer, bufferSize, offset, self->azimuth);
    offset = insert_bool(buffer, bufferSize, offset, self->health);
    offset = insert_MipData_Gnss_SatelliteStatus_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_SatelliteStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_SatelliteStatus* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipGnssConstellationId(buffer, bufferSize, offset, &self->gnss_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_float(buffer, bufferSize, offset, &self->elevation);
    offset = extract_float(buffer, bufferSize, offset, &self->azimuth);
    offset = extract_bool(buffer, bufferSize, offset, &self->health);
    offset = extract_MipData_Gnss_SatelliteStatus_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_Raw_Gnsssignalquality(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_Raw_Gnsssignalquality self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_Raw_Gnsssignalquality(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_Raw_Gnsssignalquality* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Gnss_Raw_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_Raw_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_Raw_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_Raw_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_Raw(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_Raw* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u16(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_u8(buffer, bufferSize, offset, self->tracking_channel);
    offset = insert_MipGnssConstellationId(buffer, bufferSize, offset, self->gnss_id);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_MipGnssSignalId(buffer, bufferSize, offset, self->signal_id);
    offset = insert_float(buffer, bufferSize, offset, self->signal_strength);
    offset = insert_MipData_Gnss_Raw_Gnsssignalquality(buffer, bufferSize, offset, self->quality);
    offset = insert_double(buffer, bufferSize, offset, self->pseudorange);
    offset = insert_double(buffer, bufferSize, offset, self->carrier_phase);
    offset = insert_float(buffer, bufferSize, offset, self->doppler);
    offset = insert_float(buffer, bufferSize, offset, self->range_uncert);
    offset = insert_float(buffer, bufferSize, offset, self->phase_uncert);
    offset = insert_float(buffer, bufferSize, offset, self->doppler_uncert);
    offset = insert_float(buffer, bufferSize, offset, self->lock_time);
    offset = insert_MipData_Gnss_Raw_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_Raw(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_Raw* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u16(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->tracking_channel);
    offset = extract_MipGnssConstellationId(buffer, bufferSize, offset, &self->gnss_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_MipGnssSignalId(buffer, bufferSize, offset, &self->signal_id);
    offset = extract_float(buffer, bufferSize, offset, &self->signal_strength);
    offset = extract_MipData_Gnss_Raw_Gnsssignalquality(buffer, bufferSize, offset, &self->quality);
    offset = extract_double(buffer, bufferSize, offset, &self->pseudorange);
    offset = extract_double(buffer, bufferSize, offset, &self->carrier_phase);
    offset = extract_float(buffer, bufferSize, offset, &self->doppler);
    offset = extract_float(buffer, bufferSize, offset, &self->range_uncert);
    offset = extract_float(buffer, bufferSize, offset, &self->phase_uncert);
    offset = extract_float(buffer, bufferSize, offset, &self->doppler_uncert);
    offset = extract_float(buffer, bufferSize, offset, &self->lock_time);
    offset = extract_MipData_Gnss_Raw_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_GpsEphemeris_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsEphemeris_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_GpsEphemeris_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsEphemeris_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_GpsEphemeris(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsEphemeris* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_u8(buffer, bufferSize, offset, self->health);
    offset = insert_u8(buffer, bufferSize, offset, self->iodc);
    offset = insert_u8(buffer, bufferSize, offset, self->iode);
    offset = insert_double(buffer, bufferSize, offset, self->t_oc);
    offset = insert_double(buffer, bufferSize, offset, self->af0);
    offset = insert_double(buffer, bufferSize, offset, self->af1);
    offset = insert_double(buffer, bufferSize, offset, self->af2);
    offset = insert_double(buffer, bufferSize, offset, self->t_gd);
    offset = insert_double(buffer, bufferSize, offset, self->ISC_L1CA);
    offset = insert_double(buffer, bufferSize, offset, self->ISC_L2C);
    offset = insert_double(buffer, bufferSize, offset, self->t_oe);
    offset = insert_double(buffer, bufferSize, offset, self->a);
    offset = insert_double(buffer, bufferSize, offset, self->a_dot);
    offset = insert_double(buffer, bufferSize, offset, self->mean_anomaly);
    offset = insert_double(buffer, bufferSize, offset, self->delta_mean_motion);
    offset = insert_double(buffer, bufferSize, offset, self->delta_mean_motion_dot);
    offset = insert_double(buffer, bufferSize, offset, self->eccentricity);
    offset = insert_double(buffer, bufferSize, offset, self->argument_of_perigee);
    offset = insert_double(buffer, bufferSize, offset, self->omega);
    offset = insert_double(buffer, bufferSize, offset, self->omega_dot);
    offset = insert_double(buffer, bufferSize, offset, self->inclination);
    offset = insert_double(buffer, bufferSize, offset, self->inclination_dot);
    offset = insert_double(buffer, bufferSize, offset, self->c_ic);
    offset = insert_double(buffer, bufferSize, offset, self->c_is);
    offset = insert_double(buffer, bufferSize, offset, self->c_uc);
    offset = insert_double(buffer, bufferSize, offset, self->c_us);
    offset = insert_double(buffer, bufferSize, offset, self->c_rc);
    offset = insert_double(buffer, bufferSize, offset, self->c_rs);
    offset = insert_MipData_Gnss_GpsEphemeris_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_GpsEphemeris(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsEphemeris* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_u8(buffer, bufferSize, offset, &self->health);
    offset = extract_u8(buffer, bufferSize, offset, &self->iodc);
    offset = extract_u8(buffer, bufferSize, offset, &self->iode);
    offset = extract_double(buffer, bufferSize, offset, &self->t_oc);
    offset = extract_double(buffer, bufferSize, offset, &self->af0);
    offset = extract_double(buffer, bufferSize, offset, &self->af1);
    offset = extract_double(buffer, bufferSize, offset, &self->af2);
    offset = extract_double(buffer, bufferSize, offset, &self->t_gd);
    offset = extract_double(buffer, bufferSize, offset, &self->ISC_L1CA);
    offset = extract_double(buffer, bufferSize, offset, &self->ISC_L2C);
    offset = extract_double(buffer, bufferSize, offset, &self->t_oe);
    offset = extract_double(buffer, bufferSize, offset, &self->a);
    offset = extract_double(buffer, bufferSize, offset, &self->a_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->mean_anomaly);
    offset = extract_double(buffer, bufferSize, offset, &self->delta_mean_motion);
    offset = extract_double(buffer, bufferSize, offset, &self->delta_mean_motion_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->eccentricity);
    offset = extract_double(buffer, bufferSize, offset, &self->argument_of_perigee);
    offset = extract_double(buffer, bufferSize, offset, &self->omega);
    offset = extract_double(buffer, bufferSize, offset, &self->omega_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->inclination);
    offset = extract_double(buffer, bufferSize, offset, &self->inclination_dot);
    offset = extract_double(buffer, bufferSize, offset, &self->c_ic);
    offset = extract_double(buffer, bufferSize, offset, &self->c_is);
    offset = extract_double(buffer, bufferSize, offset, &self->c_uc);
    offset = extract_double(buffer, bufferSize, offset, &self->c_us);
    offset = extract_double(buffer, bufferSize, offset, &self->c_rc);
    offset = extract_double(buffer, bufferSize, offset, &self->c_rs);
    offset = extract_MipData_Gnss_GpsEphemeris_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_GloEphemeris_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GloEphemeris_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_GloEphemeris_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GloEphemeris_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_GloEphemeris(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GloEphemeris* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->index);
    offset = insert_u8(buffer, bufferSize, offset, self->count);
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u8(buffer, bufferSize, offset, self->satellite_id);
    offset = insert_s8(buffer, bufferSize, offset, self->freq_number);
    offset = insert_u32(buffer, bufferSize, offset, self->tk);
    offset = insert_u32(buffer, bufferSize, offset, self->tb);
    offset = insert_u8(buffer, bufferSize, offset, self->sat_type);
    offset = insert_double(buffer, bufferSize, offset, self->gamma);
    offset = insert_double(buffer, bufferSize, offset, self->tau_n);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->x[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->v[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->a[i]);
    offset = insert_u8(buffer, bufferSize, offset, self->health);
    offset = insert_u8(buffer, bufferSize, offset, self->P);
    offset = insert_u8(buffer, bufferSize, offset, self->NT);
    offset = insert_float(buffer, bufferSize, offset, self->delta_tau_n);
    offset = insert_u8(buffer, bufferSize, offset, self->Ft);
    offset = insert_u8(buffer, bufferSize, offset, self->En);
    offset = insert_u8(buffer, bufferSize, offset, self->P1);
    offset = insert_u8(buffer, bufferSize, offset, self->P2);
    offset = insert_u8(buffer, bufferSize, offset, self->P3);
    offset = insert_u8(buffer, bufferSize, offset, self->P4);
    offset = insert_MipData_Gnss_GloEphemeris_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_GloEphemeris(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GloEphemeris* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->index);
    offset = extract_u8(buffer, bufferSize, offset, &self->count);
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u8(buffer, bufferSize, offset, &self->satellite_id);
    offset = extract_s8(buffer, bufferSize, offset, &self->freq_number);
    offset = extract_u32(buffer, bufferSize, offset, &self->tk);
    offset = extract_u32(buffer, bufferSize, offset, &self->tb);
    offset = extract_u8(buffer, bufferSize, offset, &self->sat_type);
    offset = extract_double(buffer, bufferSize, offset, &self->gamma);
    offset = extract_double(buffer, bufferSize, offset, &self->tau_n);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->x[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->v[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->a[i]);
    offset = extract_u8(buffer, bufferSize, offset, &self->health);
    offset = extract_u8(buffer, bufferSize, offset, &self->P);
    offset = extract_u8(buffer, bufferSize, offset, &self->NT);
    offset = extract_float(buffer, bufferSize, offset, &self->delta_tau_n);
    offset = extract_u8(buffer, bufferSize, offset, &self->Ft);
    offset = extract_u8(buffer, bufferSize, offset, &self->En);
    offset = extract_u8(buffer, bufferSize, offset, &self->P1);
    offset = extract_u8(buffer, bufferSize, offset, &self->P2);
    offset = extract_u8(buffer, bufferSize, offset, &self->P3);
    offset = extract_u8(buffer, bufferSize, offset, &self->P4);
    offset = extract_MipData_Gnss_GloEphemeris_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_GpsIonoCorr_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GpsIonoCorr_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_GpsIonoCorr_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GpsIonoCorr_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_GpsIonoCorr(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GpsIonoCorr* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_double(buffer, bufferSize, offset, self->alpha[i]);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_double(buffer, bufferSize, offset, self->beta[i]);
    offset = insert_MipData_Gnss_GpsIonoCorr_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_GpsIonoCorr(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GpsIonoCorr* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->alpha[i]);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->beta[i]);
    offset = extract_MipData_Gnss_GpsIonoCorr_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Gnss_GalileoIonoCorr_Validflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Gnss_GalileoIonoCorr_Validflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Gnss_GalileoIonoCorr_Validflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Gnss_GalileoIonoCorr_Validflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Gnss_GalileoIonoCorr(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Gnss_GalileoIonoCorr* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->alpha[i]);
    offset = insert_u8(buffer, bufferSize, offset, self->disturbance_flags);
    offset = insert_MipData_Gnss_GalileoIonoCorr_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Gnss_GalileoIonoCorr(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Gnss_GalileoIonoCorr* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->alpha[i]);
    offset = extract_u8(buffer, bufferSize, offset, &self->disturbance_flags);
    offset = extract_MipData_Gnss_GalileoIonoCorr_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
