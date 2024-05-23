
#include "data_sensor.hpp"

#include "microstrain/common/buffer.hpp"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
;

namespace C {
struct mip_interface;
} // namespace C

namespace data_sensor {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(::microstrain::Buffer& serializer, const RawAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_accel[i]);
    
}
void extract(::microstrain::Buffer& serializer, RawAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_accel[i]);
    
}

void insert(::microstrain::Buffer& serializer, const RawGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_gyro[i]);
    
}
void extract(::microstrain::Buffer& serializer, RawGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_gyro[i]);
    
}

void insert(::microstrain::Buffer& serializer, const RawMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_mag[i]);
    
}
void extract(::microstrain::Buffer& serializer, RawMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_mag[i]);
    
}

void insert(::microstrain::Buffer& serializer, const RawPressure& self)
{
    insert(serializer, self.raw_pressure);
    
}
void extract(::microstrain::Buffer& serializer, RawPressure& self)
{
    extract(serializer, self.raw_pressure);
    
}

void insert(::microstrain::Buffer& serializer, const ScaledAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_accel[i]);
    
}
void extract(::microstrain::Buffer& serializer, ScaledAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_accel[i]);
    
}

void insert(::microstrain::Buffer& serializer, const ScaledGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_gyro[i]);
    
}
void extract(::microstrain::Buffer& serializer, ScaledGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_gyro[i]);
    
}

void insert(::microstrain::Buffer& serializer, const ScaledMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_mag[i]);
    
}
void extract(::microstrain::Buffer& serializer, ScaledMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_mag[i]);
    
}

void insert(::microstrain::Buffer& serializer, const ScaledPressure& self)
{
    insert(serializer, self.scaled_pressure);
    
}
void extract(::microstrain::Buffer& serializer, ScaledPressure& self)
{
    extract(serializer, self.scaled_pressure);
    
}

void insert(::microstrain::Buffer& serializer, const DeltaTheta& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.delta_theta[i]);
    
}
void extract(::microstrain::Buffer& serializer, DeltaTheta& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.delta_theta[i]);
    
}

void insert(::microstrain::Buffer& serializer, const DeltaVelocity& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.delta_velocity[i]);
    
}
void extract(::microstrain::Buffer& serializer, DeltaVelocity& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.delta_velocity[i]);
    
}

void insert(::microstrain::Buffer& serializer, const CompOrientationMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.m[i]);
    
}
void extract(::microstrain::Buffer& serializer, CompOrientationMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.m[i]);
    
}

void insert(::microstrain::Buffer& serializer, const CompQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
    
}
void extract(::microstrain::Buffer& serializer, CompQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
    
}

void insert(::microstrain::Buffer& serializer, const CompEulerAngles& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(::microstrain::Buffer& serializer, CompEulerAngles& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.yaw);
    
}

void insert(::microstrain::Buffer& serializer, const CompOrientationUpdateMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.m[i]);
    
}
void extract(::microstrain::Buffer& serializer, CompOrientationUpdateMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.m[i]);
    
}

void insert(::microstrain::Buffer& serializer, const OrientationRawTemp& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.raw_temp[i]);
    
}
void extract(::microstrain::Buffer& serializer, OrientationRawTemp& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.raw_temp[i]);
    
}

void insert(::microstrain::Buffer& serializer, const InternalTimestamp& self)
{
    insert(serializer, self.counts);
    
}
void extract(::microstrain::Buffer& serializer, InternalTimestamp& self)
{
    extract(serializer, self.counts);
    
}

void insert(::microstrain::Buffer& serializer, const PpsTimestamp& self)
{
    insert(serializer, self.seconds);
    
    insert(serializer, self.useconds);
    
}
void extract(::microstrain::Buffer& serializer, PpsTimestamp& self)
{
    extract(serializer, self.seconds);
    
    extract(serializer, self.useconds);
    
}

void insert(::microstrain::Buffer& serializer, const GpsTimestamp& self)
{
    insert(serializer, self.tow);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.valid_flags);
    
}
void extract(::microstrain::Buffer& serializer, GpsTimestamp& self)
{
    extract(serializer, self.tow);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.valid_flags);
    
}

void insert(::microstrain::Buffer& serializer, const TemperatureAbs& self)
{
    insert(serializer, self.min_temp);
    
    insert(serializer, self.max_temp);
    
    insert(serializer, self.mean_temp);
    
}
void extract(::microstrain::Buffer& serializer, TemperatureAbs& self)
{
    extract(serializer, self.min_temp);
    
    extract(serializer, self.max_temp);
    
    extract(serializer, self.mean_temp);
    
}

void insert(::microstrain::Buffer& serializer, const UpVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.up[i]);
    
}
void extract(::microstrain::Buffer& serializer, UpVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.up[i]);
    
}

void insert(::microstrain::Buffer& serializer, const NorthVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.north[i]);
    
}
void extract(::microstrain::Buffer& serializer, NorthVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.north[i]);
    
}

void insert(::microstrain::Buffer& serializer, const OverrangeStatus& self)
{
    insert(serializer, self.status);
    
}
void extract(::microstrain::Buffer& serializer, OverrangeStatus& self)
{
    extract(serializer, self.status);
    
}

void insert(::microstrain::Buffer& serializer, const OdometerData& self)
{
    insert(serializer, self.speed);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(::microstrain::Buffer& serializer, OdometerData& self)
{
    extract(serializer, self.speed);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}


} // namespace data_sensor
} // namespace mip

