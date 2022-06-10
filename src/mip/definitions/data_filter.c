
#include "data_filter.h"

#include "utils/serialization.h"

#include <assert.h>


#ifdef __cplusplus
namespace mscl {
extern "C" {
#endif // __cplusplus


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

size_t insert_MipFilterMode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterMode self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipFilterMode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterMode* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipFilterDynamicsMode(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterDynamicsMode self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipFilterDynamicsMode(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterDynamicsMode* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipFilterStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterStatusFlags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipFilterStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterStatusFlags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipFilterAidingMeasurementType(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterAidingMeasurementType self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipFilterAidingMeasurementType(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterAidingMeasurementType* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipFilterMeasurementIndicator(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipFilterMeasurementIndicator self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipFilterMeasurementIndicator(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipFilterMeasurementIndicator* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipGnssAidStatusFlags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipGnssAidStatusFlags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipGnssAidStatusFlags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipGnssAidStatusFlags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}



////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_LlhPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_LlhPos* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->latitude);
    offset = insert_double(buffer, bufferSize, offset, self->longitude);
    offset = insert_double(buffer, bufferSize, offset, self->ellipsoid_height);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_LlhPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_LlhPos* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->latitude);
    offset = extract_double(buffer, bufferSize, offset, &self->longitude);
    offset = extract_double(buffer, bufferSize, offset, &self->ellipsoid_height);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_NedVelocity(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_NedVelocity* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->north);
    offset = insert_float(buffer, bufferSize, offset, self->east);
    offset = insert_float(buffer, bufferSize, offset, self->down);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_NedVelocity(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_NedVelocity* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->north);
    offset = extract_float(buffer, bufferSize, offset, &self->east);
    offset = extract_float(buffer, bufferSize, offset, &self->down);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AttitudeQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AttitudeQuaternion* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AttitudeQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AttitudeQuaternion* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AttitudeDcm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AttitudeDcm* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AttitudeDcm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AttitudeDcm* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_EulerAngles(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EulerAngles* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_EulerAngles(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EulerAngles* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GyroBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroBias* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_GyroBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroBias* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AccelBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelBias* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AccelBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelBias* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_LlhPosUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_LlhPosUncertainty* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->north);
    offset = insert_float(buffer, bufferSize, offset, self->east);
    offset = insert_float(buffer, bufferSize, offset, self->down);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_LlhPosUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_LlhPosUncertainty* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->north);
    offset = extract_float(buffer, bufferSize, offset, &self->east);
    offset = extract_float(buffer, bufferSize, offset, &self->down);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_NedVelUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_NedVelUncertainty* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->north);
    offset = insert_float(buffer, bufferSize, offset, self->east);
    offset = insert_float(buffer, bufferSize, offset, self->down);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_NedVelUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_NedVelUncertainty* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->north);
    offset = extract_float(buffer, bufferSize, offset, &self->east);
    offset = extract_float(buffer, bufferSize, offset, &self->down);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_EulerAnglesUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EulerAnglesUncertainty* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_EulerAnglesUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EulerAnglesUncertainty* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GyroBiasUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroBiasUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_GyroBiasUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroBiasUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AccelBiasUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelBiasUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AccelBiasUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelBiasUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_Timestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_Timestamp* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_Timestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_Timestamp* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_Status(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_Status* self)
{
    offset = insert_MipFilterMode(buffer, bufferSize, offset, self->filter_state);
    offset = insert_MipFilterDynamicsMode(buffer, bufferSize, offset, self->dynamics_mode);
    offset = insert_MipFilterStatusFlags(buffer, bufferSize, offset, self->status_flags);
    
    return offset;
}

size_t extract_MipData_Filter_Status(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_Status* self)
{
    offset = extract_MipFilterMode(buffer, bufferSize, offset, &self->filter_state);
    offset = extract_MipFilterDynamicsMode(buffer, bufferSize, offset, &self->dynamics_mode);
    offset = extract_MipFilterStatusFlags(buffer, bufferSize, offset, &self->status_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_LinearAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_LinearAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->accel[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_LinearAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_LinearAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->accel[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GravityVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GravityVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->gravity[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_GravityVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GravityVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->gravity[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_CompAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_CompAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->accel[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_CompAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_CompAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->accel[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_CompAngularRate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_CompAngularRate* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->gyro[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_CompAngularRate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_CompAngularRate* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->gyro[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_QuaternionAttitudeUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_QuaternionAttitudeUncertainty* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_QuaternionAttitudeUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_QuaternionAttitudeUncertainty* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_Wgs84GravityMag(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_Wgs84GravityMag* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->magnitude);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_Wgs84GravityMag(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_Wgs84GravityMag* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->magnitude);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_HeadingUpdateState_Headingsource(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_HeadingUpdateState_Headingsource self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Filter_HeadingUpdateState_Headingsource(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_HeadingUpdateState_Headingsource* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Filter_HeadingUpdateState(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_HeadingUpdateState* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_1sigma);
    offset = insert_MipData_Filter_HeadingUpdateState_Headingsource(buffer, bufferSize, offset, self->source);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_HeadingUpdateState(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_HeadingUpdateState* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_1sigma);
    offset = extract_MipData_Filter_HeadingUpdateState_Headingsource(buffer, bufferSize, offset, &self->source);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagneticModel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagneticModel* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->intensity_north);
    offset = insert_float(buffer, bufferSize, offset, self->intensity_east);
    offset = insert_float(buffer, bufferSize, offset, self->intensity_down);
    offset = insert_float(buffer, bufferSize, offset, self->inclination);
    offset = insert_float(buffer, bufferSize, offset, self->declination);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagneticModel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagneticModel* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->intensity_north);
    offset = extract_float(buffer, bufferSize, offset, &self->intensity_east);
    offset = extract_float(buffer, bufferSize, offset, &self->intensity_down);
    offset = extract_float(buffer, bufferSize, offset, &self->inclination);
    offset = extract_float(buffer, bufferSize, offset, &self->declination);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AccelScaleFactor(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelScaleFactor* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AccelScaleFactor(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelScaleFactor* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AccelScaleFactorUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AccelScaleFactorUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AccelScaleFactorUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AccelScaleFactorUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GyroScaleFactor(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroScaleFactor* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_GyroScaleFactor(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroScaleFactor* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GyroScaleFactorUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GyroScaleFactorUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scale_factor_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_GyroScaleFactorUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GyroScaleFactorUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagBias(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagBias* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagBias(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagBias* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagBiasUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagBiasUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->bias_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagBiasUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagBiasUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->bias_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_StandardAtmosphere(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_StandardAtmosphere* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->geometric_altitude);
    offset = insert_float(buffer, bufferSize, offset, self->geopotential_altitude);
    offset = insert_float(buffer, bufferSize, offset, self->standard_temperature);
    offset = insert_float(buffer, bufferSize, offset, self->standard_pressure);
    offset = insert_float(buffer, bufferSize, offset, self->standard_density);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_StandardAtmosphere(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_StandardAtmosphere* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->geometric_altitude);
    offset = extract_float(buffer, bufferSize, offset, &self->geopotential_altitude);
    offset = extract_float(buffer, bufferSize, offset, &self->standard_temperature);
    offset = extract_float(buffer, bufferSize, offset, &self->standard_pressure);
    offset = extract_float(buffer, bufferSize, offset, &self->standard_density);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_PressureAltitude(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_PressureAltitude* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->pressure_altitude);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_PressureAltitude(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_PressureAltitude* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->pressure_altitude);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_DensityAltitude(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_DensityAltitude* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->density_altitude);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_DensityAltitude(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_DensityAltitude* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->density_altitude);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AntennaOffsetCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AntennaOffsetCorrection* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AntennaOffsetCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AntennaOffsetCorrection* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AntennaOffsetCorrectionUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AntennaOffsetCorrectionUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_AntennaOffsetCorrectionUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AntennaOffsetCorrectionUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MultiAntennaOffsetCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MultiAntennaOffsetCorrection* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MultiAntennaOffsetCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MultiAntennaOffsetCorrection* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MultiAntennaOffsetCorrectionUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MultiAntennaOffsetCorrectionUncertainty* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset_uncert[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MultiAntennaOffsetCorrectionUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MultiAntennaOffsetCorrectionUncertainty* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset_uncert[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagnetometerOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerOffset* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->hard_iron[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagnetometerOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerOffset* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->hard_iron[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagnetometerMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->soft_iron[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagnetometerMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->soft_iron[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagnetometerOffsetUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerOffsetUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->hard_iron_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagnetometerOffsetUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerOffsetUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->hard_iron_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagnetometerMatrixUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerMatrixUncertainty* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->soft_iron_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagnetometerMatrixUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerMatrixUncertainty* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->soft_iron_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagnetometerCovarianceMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerCovarianceMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->covariance[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagnetometerCovarianceMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerCovarianceMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->covariance[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_MagnetometerResidualVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_MagnetometerResidualVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->residual[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_MagnetometerResidualVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_MagnetometerResidualVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->residual[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_ClockCorrection(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_ClockCorrection* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_float(buffer, bufferSize, offset, self->bias);
    offset = insert_float(buffer, bufferSize, offset, self->bias_drift);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_ClockCorrection(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_ClockCorrection* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_float(buffer, bufferSize, offset, &self->bias);
    offset = extract_float(buffer, bufferSize, offset, &self->bias_drift);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_ClockCorrectionUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_ClockCorrectionUncertainty* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_float(buffer, bufferSize, offset, self->bias_uncertainty);
    offset = insert_float(buffer, bufferSize, offset, self->bias_drift_uncertainty);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_ClockCorrectionUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_ClockCorrectionUncertainty* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_float(buffer, bufferSize, offset, &self->bias_uncertainty);
    offset = extract_float(buffer, bufferSize, offset, &self->bias_drift_uncertainty);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GnssPosAidStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GnssPosAidStatus* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_MipGnssAidStatusFlags(buffer, bufferSize, offset, self->status);
    
    assert(8 <= 8);
    for(unsigned int i=0; i < 8; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipData_Filter_GnssPosAidStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GnssPosAidStatus* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_MipGnssAidStatusFlags(buffer, bufferSize, offset, &self->status);
    
    assert(8 <= 8);
    for(unsigned int i=0; i < 8; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GnssAttAidStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GnssAttAidStatus* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_MipGnssAidStatusFlags(buffer, bufferSize, offset, self->status);
    
    assert(8 <= 8);
    for(unsigned int i=0; i < 8; i++)
        offset = insert_u8(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipData_Filter_GnssAttAidStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GnssAttAidStatus* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_MipGnssAidStatusFlags(buffer, bufferSize, offset, &self->status);
    
    assert(8 <= 8);
    for(unsigned int i=0; i < 8; i++)
        offset = extract_u8(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_HeadAidStatus_Headingaidtype(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_HeadAidStatus_Headingaidtype self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Filter_HeadAidStatus_Headingaidtype(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_HeadAidStatus_Headingaidtype* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Filter_HeadAidStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_HeadAidStatus* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_MipData_Filter_HeadAidStatus_Headingaidtype(buffer, bufferSize, offset, self->type);
    
    assert(2 <= 2);
    for(unsigned int i=0; i < 2; i++)
        offset = insert_float(buffer, bufferSize, offset, self->reserved[i]);
    
    return offset;
}

size_t extract_MipData_Filter_HeadAidStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_HeadAidStatus* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_MipData_Filter_HeadAidStatus_Headingaidtype(buffer, bufferSize, offset, &self->type);
    
    assert(2 <= 2);
    for(unsigned int i=0; i < 2; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->reserved[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_RelPosNed(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_RelPosNed* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->relative_position[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_RelPosNed(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_RelPosNed* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->relative_position[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_EcefPos(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefPos* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->position_ecef[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_EcefPos(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefPos* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->position_ecef[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_EcefVel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefVel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->velocity_ecef[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_EcefVel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefVel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->velocity_ecef[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_EcefPosUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefPosUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->pos_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_EcefPosUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefPosUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->pos_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_EcefVelUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_EcefVelUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->vel_uncertainty[i]);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_EcefVelUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_EcefVelUncertainty* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->vel_uncertainty[i]);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_AidingMeasurementSummary(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_AidingMeasurementSummary* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_MipFilterAidingMeasurementType(buffer, bufferSize, offset, self->type);
    offset = insert_MipFilterMeasurementIndicator(buffer, bufferSize, offset, self->indicator);
    
    return offset;
}

size_t extract_MipData_Filter_AidingMeasurementSummary(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_AidingMeasurementSummary* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_MipFilterAidingMeasurementType(buffer, bufferSize, offset, &self->type);
    offset = extract_MipFilterMeasurementIndicator(buffer, bufferSize, offset, &self->indicator);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_OdometerScaleFactorError(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_OdometerScaleFactorError* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->scale_factor_error);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_OdometerScaleFactorError(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_OdometerScaleFactorError* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_error);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_OdometerScaleFactorErrorUncertainty(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_OdometerScaleFactorErrorUncertainty* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->scale_factor_error_uncertainty);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_OdometerScaleFactorErrorUncertainty(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_OdometerScaleFactorErrorUncertainty* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->scale_factor_error_uncertainty);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Filter_GnssDualAntennaStatus_Fixtype(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_GnssDualAntennaStatus_Fixtype self)
{
    return insert_u8(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Filter_GnssDualAntennaStatus_Fixtype(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_GnssDualAntennaStatus_Fixtype* self)
{
    uint8_t tmp;
    offset = extract_u8(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}

size_t insert_MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags(uint8_t* buffer, size_t bufferSize, size_t offset, const enum MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags self)
{
    return insert_u16(buffer, bufferSize, offset, self);
}
size_t extract_MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags(const uint8_t* buffer, size_t bufferSize, size_t offset, enum MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags* self)
{
    uint16_t tmp;
    offset = extract_u16(buffer, bufferSize, offset, &tmp);
    *self = tmp;
    return offset;
}


size_t insert_MipData_Filter_GnssDualAntennaStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Filter_GnssDualAntennaStatus* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_unc);
    offset = insert_MipData_Filter_GnssDualAntennaStatus_Fixtype(buffer, bufferSize, offset, self->fix_type);
    offset = insert_MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags(buffer, bufferSize, offset, self->status_flags);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Filter_GnssDualAntennaStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Filter_GnssDualAntennaStatus* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_unc);
    offset = extract_MipData_Filter_GnssDualAntennaStatus_Fixtype(buffer, bufferSize, offset, &self->fix_type);
    offset = extract_MipData_Filter_GnssDualAntennaStatus_Dualantennastatusflags(buffer, bufferSize, offset, &self->status_flags);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}



#ifdef __cplusplus
} // extern "C"
} // namespace mscl
#endif // __cplusplus
