
#include "data_sensor.h"

#include "utils/serialization.h"

#include <assert.h>



////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_RawAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->raw_accel[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_RawAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->raw_accel[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_RawGyro(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawGyro* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->raw_gyro[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_RawGyro(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawGyro* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->raw_gyro[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_RawMag(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawMag* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->raw_mag[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_RawMag(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawMag* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->raw_mag[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_RawPressure(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_RawPressure* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->raw_pressure);
    
    return offset;
}

size_t extract_MipData_Sensor_RawPressure(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_RawPressure* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->raw_pressure);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_ScaledAccel(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scaled_accel[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_ScaledAccel(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledAccel* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scaled_accel[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_ScaledGyro(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledGyro* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scaled_gyro[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_ScaledGyro(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledGyro* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scaled_gyro[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_ScaledMag(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledMag* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->scaled_mag[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_ScaledMag(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledMag* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->scaled_mag[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_ScaledPressure(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_ScaledPressure* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->scaled_pressure);
    
    return offset;
}

size_t extract_MipData_Sensor_ScaledPressure(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_ScaledPressure* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->scaled_pressure);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_DeltaTheta(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_DeltaTheta* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->delta_theta[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_DeltaTheta(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_DeltaTheta* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->delta_theta[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_DeltaVelocity(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_DeltaVelocity* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->delta_velocity[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_DeltaVelocity(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_DeltaVelocity* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->delta_velocity[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_CompOrientationMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompOrientationMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->m[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_CompOrientationMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompOrientationMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->m[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_CompQuaternion(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompQuaternion* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_float(buffer, bufferSize, offset, self->q[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_CompQuaternion(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompQuaternion* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->q[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_CompEulerAngles(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompEulerAngles* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->roll);
    offset = insert_float(buffer, bufferSize, offset, self->pitch);
    offset = insert_float(buffer, bufferSize, offset, self->yaw);
    
    return offset;
}

size_t extract_MipData_Sensor_CompEulerAngles(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompEulerAngles* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->roll);
    offset = extract_float(buffer, bufferSize, offset, &self->pitch);
    offset = extract_float(buffer, bufferSize, offset, &self->yaw);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_CompOrientationUpdateMatrix(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_CompOrientationUpdateMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = insert_float(buffer, bufferSize, offset, self->m[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_CompOrientationUpdateMatrix(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_CompOrientationUpdateMatrix* self)
{
    
    assert(9 <= 9);
    for(unsigned int i=0; i < 9; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->m[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_OrientationRawTemp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_OrientationRawTemp* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = insert_u16(buffer, bufferSize, offset, self->raw_temp[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_OrientationRawTemp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_OrientationRawTemp* self)
{
    
    assert(4 <= 4);
    for(unsigned int i=0; i < 4; i++)
        offset = extract_u16(buffer, bufferSize, offset, &self->raw_temp[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_InternalTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_InternalTimestamp* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->counts);
    
    return offset;
}

size_t extract_MipData_Sensor_InternalTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_InternalTimestamp* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->counts);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_1ppsTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_1ppsTimestamp* self)
{
    offset = insert_u32(buffer, bufferSize, offset, self->seconds);
    offset = insert_u32(buffer, bufferSize, offset, self->useconds);
    
    return offset;
}

size_t extract_MipData_Sensor_1ppsTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_1ppsTimestamp* self)
{
    offset = extract_u32(buffer, bufferSize, offset, &self->seconds);
    offset = extract_u32(buffer, bufferSize, offset, &self->useconds);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_GpsTimestamp(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_GpsTimestamp* self)
{
    offset = insert_double(buffer, bufferSize, offset, self->tow);
    offset = insert_u16(buffer, bufferSize, offset, self->week_number);
    offset = insert_MipData_Sensor_GpsTimestamp_Validflags(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Sensor_GpsTimestamp(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_GpsTimestamp* self)
{
    offset = extract_double(buffer, bufferSize, offset, &self->tow);
    offset = extract_u16(buffer, bufferSize, offset, &self->week_number);
    offset = extract_MipData_Sensor_GpsTimestamp_Validflags(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_TemperatureAbs(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_TemperatureAbs* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->min_temp);
    offset = insert_float(buffer, bufferSize, offset, self->max_temp);
    offset = insert_float(buffer, bufferSize, offset, self->mean_temp);
    
    return offset;
}

size_t extract_MipData_Sensor_TemperatureAbs(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_TemperatureAbs* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->min_temp);
    offset = extract_float(buffer, bufferSize, offset, &self->max_temp);
    offset = extract_float(buffer, bufferSize, offset, &self->mean_temp);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_UpVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_UpVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->up[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_UpVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_UpVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->up[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_NorthVector(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_NorthVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = insert_float(buffer, bufferSize, offset, self->north[i]);
    
    return offset;
}

size_t extract_MipData_Sensor_NorthVector(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_NorthVector* self)
{
    
    assert(3 <= 3);
    for(unsigned int i=0; i < 3; i++)
        offset = extract_float(buffer, bufferSize, offset, &self->north[i]);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_OverrangeStatus(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_OverrangeStatus* self)
{
    offset = insert_MipData_Sensor_OverrangeStatus_Status(buffer, bufferSize, offset, self->status);
    
    return offset;
}

size_t extract_MipData_Sensor_OverrangeStatus(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_OverrangeStatus* self)
{
    offset = extract_MipData_Sensor_OverrangeStatus_Status(buffer, bufferSize, offset, &self->status);
    
    return offset;
}


////////////////////////////////////////////////////////////////////////////////
size_t insert_MipData_Sensor_OdometerData(uint8_t* buffer, size_t bufferSize, size_t offset, const struct MipData_Sensor_OdometerData* self)
{
    offset = insert_float(buffer, bufferSize, offset, self->speed);
    offset = insert_float(buffer, bufferSize, offset, self->uncertainty);
    offset = insert_u16(buffer, bufferSize, offset, self->valid_flags);
    
    return offset;
}

size_t extract_MipData_Sensor_OdometerData(const uint8_t* buffer, size_t bufferSize, size_t offset, struct MipData_Sensor_OdometerData* self)
{
    offset = extract_float(buffer, bufferSize, offset, &self->speed);
    offset = extract_float(buffer, bufferSize, offset, &self->uncertainty);
    offset = extract_u16(buffer, bufferSize, offset, &self->valid_flags);
    
    return offset;
}


