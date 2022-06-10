
#include "commands_filter.h"

#include "utils/serialization.h"

#include <assert.h>



////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_ResetFilter(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ResetFilter* self)
{
    return offset;
}

size_t extract_MipCmd_Filter_ResetFilter(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ResetFilter* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_SetInitialAttitude(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SetInitialAttitude* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    
    return offset;
}

size_t extract_MipCmd_Filter_SetInitialAttitude(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SetInitialAttitude* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_EstimationControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_EstimationControl* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_MipCmd_Filter_EstimationControl_Enableflags(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_EstimationControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_EstimationControl* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_MipCmd_Filter_EstimationControl_Enableflags(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_MipCmd_Filter_EstimationControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_EstimationControl_Response* self)
{
    offset = insert_MipCmd_Filter_EstimationControl_Enableflags(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_EstimationControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_EstimationControl_Response* self)
{
    offset = extract_MipCmd_Filter_EstimationControl_Enableflags(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_ExternalGnssUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ExternalGnssUpdate* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->gps_time);
    offset = insert_u16(buffer, bufferSize, offset, self->gps_week);
    offset = insert_double(buffer, bufferSize, offset, self->latitude);
    offset = insert_double(buffer, bufferSize, offset, self->longitude);
    offset = insert_double(buffer, bufferSize, offset, self->height);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->velocity[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->pos_uncertainty[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->vel_uncertainty[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_ExternalGnssUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ExternalGnssUpdate* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->gps_time);
    offset = extract_u16(buffer, bufferSize, offset, &self->gps_week);
    offset = extract_double(buffer, bufferSize, offset, &self->latitude);
    offset = extract_double(buffer, bufferSize, offset, &self->longitude);
    offset = extract_double(buffer, bufferSize, offset, &self->height);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->velocity[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->pos_uncertainty[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->vel_uncertainty[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_ExternalHeadingUpdate(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ExternalHeadingUpdate* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_uncertainty);
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    
    return offset;
}

size_t extract_MipCmd_Filter_ExternalHeadingUpdate(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ExternalHeadingUpdate* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_uncertainty);
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_ExternalHeadingUpdateWithTime(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_ExternalHeadingUpdateWithTime* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->gps_time);
    offset = insert_u16(buffer, bufferSize, offset, self->gps_week);
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    offset = insert_float(buffer, bufferSize, offset, self->heading_uncertainty);
    offset = insert_u8(buffer, bufferSize, offset, self->type);
    
    return offset;
}

size_t extract_MipCmd_Filter_ExternalHeadingUpdateWithTime(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_ExternalHeadingUpdateWithTime* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->gps_time);
    offset = extract_u16(buffer, bufferSize, offset, &self->gps_week);
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    offset = extract_float(buffer, bufferSize, offset, &self->heading_uncertainty);
    offset = extract_u8(buffer, bufferSize, offset, &self->type);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_TareOrientation(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_TareOrientation* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_MipCmd_Filter_TareOrientation_Miptareaxes(buffer, bufferSize, offset, self->axes);
    
    return offset;
}

size_t extract_MipCmd_Filter_TareOrientation(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_TareOrientation* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_MipCmd_Filter_TareOrientation_Miptareaxes(buffer, bufferSize, offset, &self->axes);
    
    return offset;
}


size_t insert_MipCmd_Filter_TareOrientation_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_TareOrientation_Response* self)
{
    offset = insert_MipCmd_Filter_TareOrientation_Miptareaxes(buffer, bufferSize, offset, self->axes);
    
    return offset;
}

size_t extract_MipCmd_Filter_TareOrientation_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_TareOrientation_Response* self)
{
    offset = extract_MipCmd_Filter_TareOrientation_Miptareaxes(buffer, bufferSize, offset, &self->axes);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_Sensor2VehicleRotationEuler(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationEuler* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleRotationEuler(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationEuler* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


size_t insert_MipCmd_Filter_Sensor2VehicleRotationEuler_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationEuler_Response* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleRotationEuler_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationEuler_Response* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_Sensor2VehicleRotationDcm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationDcm* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleRotationDcm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationDcm* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_Sensor2VehicleRotationDcm_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationDcm_Response* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->dcm[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleRotationDcm_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationDcm_Response* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->dcm[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_Sensor2VehicleRotationQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationQuaternion* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->quat[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleRotationQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationQuaternion* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->quat[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->quat[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleRotationQuaternion_Response* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->quat[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_Sensor2VehicleOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleOffset* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleOffset* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_Sensor2VehicleOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Sensor2VehicleOffset_Response* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_Sensor2VehicleOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Sensor2VehicleOffset_Response* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_AntennaOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AntennaOffset* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_AntennaOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AntennaOffset* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_AntennaOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AntennaOffset_Response* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_AntennaOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AntennaOffset_Response* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->offset[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_GnssSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssSource* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_Filter_GnssSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssSource* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


size_t insert_MipCmd_Filter_GnssSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssSource_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_Filter_GnssSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssSource_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_HeadingSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_HeadingSource* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_MipCmd_Filter_HeadingSource_Headingsource(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_Filter_HeadingSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_HeadingSource* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_MipCmd_Filter_HeadingSource_Headingsource(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


size_t insert_MipCmd_Filter_HeadingSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_HeadingSource_Response* self)
{
    offset = insert_MipCmd_Filter_HeadingSource_Headingsource(buffer, bufferSize, offset, self->source);
    
    return offset;
}

size_t extract_MipCmd_Filter_HeadingSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_HeadingSource_Response* self)
{
    offset = extract_MipCmd_Filter_HeadingSource_Headingsource(buffer, bufferSize, offset, &self->source);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_AltitudeAiding(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AltitudeAiding* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->aiding_selector);
    
    return offset;
}

size_t extract_MipCmd_Filter_AltitudeAiding(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AltitudeAiding* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->aiding_selector);
    
    return offset;
}


size_t insert_MipCmd_Filter_AltitudeAiding_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AltitudeAiding_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->aiding_selector);
    
    return offset;
}

size_t extract_MipCmd_Filter_AltitudeAiding_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AltitudeAiding_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->aiding_selector);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_AutoZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoZupt* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_MipCmd_Filter_AutoZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoZupt* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


size_t insert_MipCmd_Filter_AutoZupt_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoZupt_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_MipCmd_Filter_AutoZupt_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoZupt_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_AutoAngularZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoAngularZupt* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_MipCmd_Filter_AutoAngularZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoAngularZupt* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


size_t insert_MipCmd_Filter_AutoAngularZupt_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AutoAngularZupt_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->threshold);
    
    return offset;
}

size_t extract_MipCmd_Filter_AutoAngularZupt_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AutoAngularZupt_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->threshold);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_CommandedZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_CommandedZupt* self)
{
    return offset;
}

size_t extract_MipCmd_Filter_CommandedZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_CommandedZupt* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_CommandedAngularZupt(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_CommandedAngularZupt* self)
{
    return offset;
}

size_t extract_MipCmd_Filter_CommandedAngularZupt(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_CommandedAngularZupt* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_AidingMeasurementEnable(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AidingMeasurementEnable* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_MipCmd_Filter_AidingMeasurementEnable_Aidingsource(buffer, bufferSize, offset, self->aiding_source);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_AidingMeasurementEnable(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AidingMeasurementEnable* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_MipCmd_Filter_AidingMeasurementEnable_Aidingsource(buffer, bufferSize, offset, &self->aiding_source);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_MipCmd_Filter_AidingMeasurementEnable_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AidingMeasurementEnable_Response* self)
{
    offset = insert_MipCmd_Filter_AidingMeasurementEnable_Aidingsource(buffer, bufferSize, offset, self->aiding_source);
    offset = insert_bool(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_AidingMeasurementEnable_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AidingMeasurementEnable_Response* self)
{
    offset = extract_MipCmd_Filter_AidingMeasurementEnable_Aidingsource(buffer, bufferSize, offset, &self->aiding_source);
    offset = extract_bool(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_Run(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_Run* self)
{
    return offset;
}

size_t extract_MipCmd_Filter_Run(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_Run* self)
{
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_KinematicConstraint(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_KinematicConstraint* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->acceleration_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->velocity_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->angular_constraint_selection);
    
    return offset;
}

size_t extract_MipCmd_Filter_KinematicConstraint(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_KinematicConstraint* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->acceleration_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->velocity_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->angular_constraint_selection);
    
    return offset;
}


size_t insert_MipCmd_Filter_KinematicConstraint_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_KinematicConstraint_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->acceleration_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->velocity_constraint_selection);
    offset = insert_u8(buffer, bufferSize, offset, self->angular_constraint_selection);
    
    return offset;
}

size_t extract_MipCmd_Filter_KinematicConstraint_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_KinematicConstraint_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->acceleration_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->velocity_constraint_selection);
    offset = extract_u8(buffer, bufferSize, offset, &self->angular_constraint_selection);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_InitializationConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_InitializationConfiguration* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->wait_for_run_command);
    offset = insert_MipCmd_Filter_InitializationConfiguration_Initialconditionsource(buffer, bufferSize, offset, self->initial_cond_src);
    offset = insert_MipCmd_Filter_InitializationConfiguration_Alignmentselector(buffer, bufferSize, offset, self->auto_heading_alignment_selector);
    offset = insert_float(buffer, bufferSize, offset, self->initial_heading);
    offset = insert_float(buffer, bufferSize, offset, self->initial_pitch);
    offset = insert_float(buffer, bufferSize, offset, self->initial_roll);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_position[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_velocity[i]);
    offset = insert_MipFilterReferenceFrame(buffer, bufferSize, offset, self->reference_frame_selector);
    
    return offset;
}

size_t extract_MipCmd_Filter_InitializationConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_InitializationConfiguration* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->wait_for_run_command);
    offset = extract_MipCmd_Filter_InitializationConfiguration_Initialconditionsource(buffer, bufferSize, offset, &self->initial_cond_src);
    offset = extract_MipCmd_Filter_InitializationConfiguration_Alignmentselector(buffer, bufferSize, offset, &self->auto_heading_alignment_selector);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_heading);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_roll);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_position[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_velocity[i]);
    offset = extract_MipFilterReferenceFrame(buffer, bufferSize, offset, &self->reference_frame_selector);
    
    return offset;
}


size_t insert_MipCmd_Filter_InitializationConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_InitializationConfiguration_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->wait_for_run_command);
    offset = insert_MipCmd_Filter_InitializationConfiguration_Initialconditionsource(buffer, bufferSize, offset, self->initial_cond_src);
    offset = insert_MipCmd_Filter_InitializationConfiguration_Alignmentselector(buffer, bufferSize, offset, self->auto_heading_alignment_selector);
    offset = insert_float(buffer, bufferSize, offset, self->initial_heading);
    offset = insert_float(buffer, bufferSize, offset, self->initial_pitch);
    offset = insert_float(buffer, bufferSize, offset, self->initial_roll);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_position[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->initial_velocity[i]);
    offset = insert_MipFilterReferenceFrame(buffer, bufferSize, offset, self->reference_frame_selector);
    
    return offset;
}

size_t extract_MipCmd_Filter_InitializationConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_InitializationConfiguration_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->wait_for_run_command);
    offset = extract_MipCmd_Filter_InitializationConfiguration_Initialconditionsource(buffer, bufferSize, offset, &self->initial_cond_src);
    offset = extract_MipCmd_Filter_InitializationConfiguration_Alignmentselector(buffer, bufferSize, offset, &self->auto_heading_alignment_selector);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_heading);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->initial_roll);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_position[i]);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->initial_velocity[i]);
    offset = extract_MipFilterReferenceFrame(buffer, bufferSize, offset, &self->reference_frame_selector);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_AdaptiveFilterOptions(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AdaptiveFilterOptions* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->level);
    offset = insert_u16(buffer, bufferSize, offset, self->time_limit);
    
    return offset;
}

size_t extract_MipCmd_Filter_AdaptiveFilterOptions(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AdaptiveFilterOptions* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->level);
    offset = extract_u16(buffer, bufferSize, offset, &self->time_limit);
    
    return offset;
}


size_t insert_MipCmd_Filter_AdaptiveFilterOptions_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_AdaptiveFilterOptions_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->level);
    offset = insert_u16(buffer, bufferSize, offset, self->time_limit);
    
    return offset;
}

size_t extract_MipCmd_Filter_AdaptiveFilterOptions_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_AdaptiveFilterOptions_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->level);
    offset = extract_u16(buffer, bufferSize, offset, &self->time_limit);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_MultiAntennaOffset(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MultiAntennaOffset* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->antenna_offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_MultiAntennaOffset(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MultiAntennaOffset* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->antenna_offset[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_MultiAntennaOffset_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MultiAntennaOffset_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->antenna_offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_MultiAntennaOffset_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MultiAntennaOffset_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->receiver_id);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->antenna_offset[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_RelPosConfiguration(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_RelPosConfiguration* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_MipFilterReferenceFrame(buffer, bufferSize, offset, self->reference_frame_selector);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->reference_coordinates[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_RelPosConfiguration(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_RelPosConfiguration* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_MipFilterReferenceFrame(buffer, bufferSize, offset, &self->reference_frame_selector);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->reference_coordinates[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_RelPosConfiguration_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_RelPosConfiguration_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_MipFilterReferenceFrame(buffer, bufferSize, offset, self->reference_frame_selector);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_double(buffer, bufferSize, offset, self->reference_coordinates[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_RelPosConfiguration_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_RelPosConfiguration_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_MipFilterReferenceFrame(buffer, bufferSize, offset, &self->reference_frame_selector);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_double(buffer, bufferSize, offset, &self->reference_coordinates[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_SpeedMeasurement(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SpeedMeasurement* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    offset = insert_float(buffer, bufferSize, offset, self->time_of_week);
    offset = insert_float(buffer, bufferSize, offset, self->speed);
    offset = insert_float(buffer, bufferSize, offset, self->speed_uncertainty);
    
    return offset;
}

size_t extract_MipCmd_Filter_SpeedMeasurement(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SpeedMeasurement* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    offset = extract_float(buffer, bufferSize, offset, &self->time_of_week);
    offset = extract_float(buffer, bufferSize, offset, &self->speed);
    offset = extract_float(buffer, bufferSize, offset, &self->speed_uncertainty);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_SpeedLeverArm(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SpeedLeverArm* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->lever_arm_offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_SpeedLeverArm(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SpeedLeverArm* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->lever_arm_offset[i]);
    
    return offset;
}


size_t insert_MipCmd_Filter_SpeedLeverArm_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SpeedLeverArm_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->source);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->lever_arm_offset[i]);
    
    return offset;
}

size_t extract_MipCmd_Filter_SpeedLeverArm_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SpeedLeverArm_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->source);
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->lever_arm_offset[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_WheeledVehicleConstraintControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_WheeledVehicleConstraintControl* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_WheeledVehicleConstraintControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_WheeledVehicleConstraintControl* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_MipCmd_Filter_WheeledVehicleConstraintControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_WheeledVehicleConstraintControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_WheeledVehicleConstraintControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_WheeledVehicleConstraintControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_VerticalGyroConstraintControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_VerticalGyroConstraintControl* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_VerticalGyroConstraintControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_VerticalGyroConstraintControl* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


size_t insert_MipCmd_Filter_VerticalGyroConstraintControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_VerticalGyroConstraintControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    
    return offset;
}

size_t extract_MipCmd_Filter_VerticalGyroConstraintControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_VerticalGyroConstraintControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_GnssAntennaCalControl(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssAntennaCalControl* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->max_offset);
    
    return offset;
}

size_t extract_MipCmd_Filter_GnssAntennaCalControl(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssAntennaCalControl* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->max_offset);
    
    return offset;
}


size_t insert_MipCmd_Filter_GnssAntennaCalControl_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_GnssAntennaCalControl_Response* self)
{
    offset = insert_u8(buffer, bufferSize, offset, self->enable);
    offset = insert_float(buffer, bufferSize, offset, self->max_offset);
    
    return offset;
}

size_t extract_MipCmd_Filter_GnssAntennaCalControl_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_GnssAntennaCalControl_Response* self)
{
    offset = extract_u8(buffer, bufferSize, offset, &self->enable);
    offset = extract_float(buffer, bufferSize, offset, &self->max_offset);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_MagneticDeclinationSource(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MagneticDeclinationSource* self)
{
    offset = insert_MipFunctionSelector(buffer, bufferSize, offset, self->function);
    offset = insert_MipFilterMagDeclinationSource(buffer, bufferSize, offset, self->source);
    offset = insert_float(buffer, bufferSize, offset, self->declination);
    
    return offset;
}

size_t extract_MipCmd_Filter_MagneticDeclinationSource(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MagneticDeclinationSource* self)
{
    offset = extract_MipFunctionSelector(buffer, bufferSize, offset, &self->function);
    offset = extract_MipFilterMagDeclinationSource(buffer, bufferSize, offset, &self->source);
    offset = extract_float(buffer, bufferSize, offset, &self->declination);
    
    return offset;
}


size_t insert_MipCmd_Filter_MagneticDeclinationSource_Response(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_MagneticDeclinationSource_Response* self)
{
    offset = insert_MipFilterMagDeclinationSource(buffer, bufferSize, offset, self->source);
    offset = insert_float(buffer, bufferSize, offset, self->declination);
    
    return offset;
}

size_t extract_MipCmd_Filter_MagneticDeclinationSource_Response(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_MagneticDeclinationSource_Response* self)
{
    offset = extract_MipFilterMagDeclinationSource(buffer, bufferSize, offset, &self->source);
    offset = extract_float(buffer, bufferSize, offset, &self->declination);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipCmd_Filter_SetInitialHeading(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipCmd_Filter_SetInitialHeading* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->heading);
    
    return offset;
}

size_t extract_MipCmd_Filter_SetInitialHeading(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipCmd_Filter_SetInitialHeading* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->heading);
    
    return offset;
}


