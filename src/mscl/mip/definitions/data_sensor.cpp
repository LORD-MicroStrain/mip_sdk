
#include "data_sensor.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class MipSerializer;

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

void insert(MipSerializer& serializer, const RawAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_accel[i]);
}

void extract(MipSerializer& serializer, RawAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_accel[i]);
}

void insert(MipSerializer& serializer, const RawGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_gyro[i]);
}

void extract(MipSerializer& serializer, RawGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_gyro[i]);
}

void insert(MipSerializer& serializer, const RawMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_mag[i]);
}

void extract(MipSerializer& serializer, RawMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_mag[i]);
}

void insert(MipSerializer& serializer, const RawPressure& self)
{
    insert(serializer, self.raw_pressure);
}

void extract(MipSerializer& serializer, RawPressure& self)
{
    extract(serializer, self.raw_pressure);
}

void insert(MipSerializer& serializer, const ScaledAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_accel[i]);
}

void extract(MipSerializer& serializer, ScaledAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_accel[i]);
}

void insert(MipSerializer& serializer, const ScaledGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_gyro[i]);
}

void extract(MipSerializer& serializer, ScaledGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_gyro[i]);
}

void insert(MipSerializer& serializer, const ScaledMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_mag[i]);
}

void extract(MipSerializer& serializer, ScaledMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_mag[i]);
}

void insert(MipSerializer& serializer, const ScaledPressure& self)
{
    insert(serializer, self.scaled_pressure);
}

void extract(MipSerializer& serializer, ScaledPressure& self)
{
    extract(serializer, self.scaled_pressure);
}

void insert(MipSerializer& serializer, const DeltaTheta& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.delta_theta[i]);
}

void extract(MipSerializer& serializer, DeltaTheta& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.delta_theta[i]);
}

void insert(MipSerializer& serializer, const DeltaVelocity& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.delta_velocity[i]);
}

void extract(MipSerializer& serializer, DeltaVelocity& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.delta_velocity[i]);
}

void insert(MipSerializer& serializer, const CompOrientationMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.m[i]);
}

void extract(MipSerializer& serializer, CompOrientationMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.m[i]);
}

void insert(MipSerializer& serializer, const CompQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
}

void extract(MipSerializer& serializer, CompQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
}

void insert(MipSerializer& serializer, const CompEulerAngles& self)
{
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
}

void extract(MipSerializer& serializer, CompEulerAngles& self)
{
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
}

void insert(MipSerializer& serializer, const CompOrientationUpdateMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.m[i]);
}

void extract(MipSerializer& serializer, CompOrientationUpdateMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.m[i]);
}

void insert(MipSerializer& serializer, const OrientationRawTemp& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.raw_temp[i]);
}

void extract(MipSerializer& serializer, OrientationRawTemp& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.raw_temp[i]);
}

void insert(MipSerializer& serializer, const InternalTimestamp& self)
{
    insert(serializer, self.counts);
}

void extract(MipSerializer& serializer, InternalTimestamp& self)
{
    extract(serializer, self.counts);
}

void insert(MipSerializer& serializer, const PpsTimestamp& self)
{
    insert(serializer, self.seconds);
    insert(serializer, self.useconds);
}

void extract(MipSerializer& serializer, PpsTimestamp& self)
{
    extract(serializer, self.seconds);
    extract(serializer, self.useconds);
}

void insert(MipSerializer& serializer, const GpsTimestamp& self)
{
    insert(serializer, self.tow);
    insert(serializer, self.week_number);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GpsTimestamp& self)
{
    extract(serializer, self.tow);
    extract(serializer, self.week_number);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const TemperatureAbs& self)
{
    insert(serializer, self.min_temp);
    insert(serializer, self.max_temp);
    insert(serializer, self.mean_temp);
}

void extract(MipSerializer& serializer, TemperatureAbs& self)
{
    extract(serializer, self.min_temp);
    extract(serializer, self.max_temp);
    extract(serializer, self.mean_temp);
}

void insert(MipSerializer& serializer, const UpVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.up[i]);
}

void extract(MipSerializer& serializer, UpVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.up[i]);
}

void insert(MipSerializer& serializer, const NorthVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.north[i]);
}

void extract(MipSerializer& serializer, NorthVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.north[i]);
}

void insert(MipSerializer& serializer, const OverrangeStatus& self)
{
    insert(serializer, self.status);
}

void extract(MipSerializer& serializer, OverrangeStatus& self)
{
    extract(serializer, self.status);
}

void insert(MipSerializer& serializer, const OdometerData& self)
{
    insert(serializer, self.speed);
    insert(serializer, self.uncertainty);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, OdometerData& self)
{
    extract(serializer, self.speed);
    extract(serializer, self.uncertainty);
    extract(serializer, self.valid_flags);
}


} // namespace data_sensor
} // namespace mip

