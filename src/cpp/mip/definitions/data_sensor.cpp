
#include "data_sensor.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_sensor {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void RawAccel::insert(Serializer& serializer) const
{
    serializer.insert(raw_accel);
    
}
void RawAccel::extract(Serializer& serializer)
{
    serializer.extract(raw_accel);
    
}

void RawGyro::insert(Serializer& serializer) const
{
    serializer.insert(raw_gyro);
    
}
void RawGyro::extract(Serializer& serializer)
{
    serializer.extract(raw_gyro);
    
}

void RawMag::insert(Serializer& serializer) const
{
    serializer.insert(raw_mag);
    
}
void RawMag::extract(Serializer& serializer)
{
    serializer.extract(raw_mag);
    
}

void RawPressure::insert(Serializer& serializer) const
{
    serializer.insert(raw_pressure);
    
}
void RawPressure::extract(Serializer& serializer)
{
    serializer.extract(raw_pressure);
    
}

void ScaledAccel::insert(Serializer& serializer) const
{
    serializer.insert(scaled_accel);
    
}
void ScaledAccel::extract(Serializer& serializer)
{
    serializer.extract(scaled_accel);
    
}

void ScaledGyro::insert(Serializer& serializer) const
{
    serializer.insert(scaled_gyro);
    
}
void ScaledGyro::extract(Serializer& serializer)
{
    serializer.extract(scaled_gyro);
    
}

void ScaledMag::insert(Serializer& serializer) const
{
    serializer.insert(scaled_mag);
    
}
void ScaledMag::extract(Serializer& serializer)
{
    serializer.extract(scaled_mag);
    
}

void ScaledPressure::insert(Serializer& serializer) const
{
    serializer.insert(scaled_pressure);
    
}
void ScaledPressure::extract(Serializer& serializer)
{
    serializer.extract(scaled_pressure);
    
}

void DeltaTheta::insert(Serializer& serializer) const
{
    serializer.insert(delta_theta);
    
}
void DeltaTheta::extract(Serializer& serializer)
{
    serializer.extract(delta_theta);
    
}

void DeltaVelocity::insert(Serializer& serializer) const
{
    serializer.insert(delta_velocity);
    
}
void DeltaVelocity::extract(Serializer& serializer)
{
    serializer.extract(delta_velocity);
    
}

void CompOrientationMatrix::insert(Serializer& serializer) const
{
    serializer.insert(m);
    
}
void CompOrientationMatrix::extract(Serializer& serializer)
{
    serializer.extract(m);
    
}

void CompQuaternion::insert(Serializer& serializer) const
{
    serializer.insert(q);
    
}
void CompQuaternion::extract(Serializer& serializer)
{
    serializer.extract(q);
    
}

void CompEulerAngles::insert(Serializer& serializer) const
{
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
}
void CompEulerAngles::extract(Serializer& serializer)
{
    serializer.extract(roll);
    
    serializer.extract(pitch);
    
    serializer.extract(yaw);
    
}

void CompOrientationUpdateMatrix::insert(Serializer& serializer) const
{
    serializer.insert(m);
    
}
void CompOrientationUpdateMatrix::extract(Serializer& serializer)
{
    serializer.extract(m);
    
}

void OrientationRawTemp::insert(Serializer& serializer) const
{
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(raw_temp[i]);
    
}
void OrientationRawTemp::extract(Serializer& serializer)
{
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(raw_temp[i]);
    
}

void InternalTimestamp::insert(Serializer& serializer) const
{
    serializer.insert(counts);
    
}
void InternalTimestamp::extract(Serializer& serializer)
{
    serializer.extract(counts);
    
}

void PpsTimestamp::insert(Serializer& serializer) const
{
    serializer.insert(seconds);
    
    serializer.insert(useconds);
    
}
void PpsTimestamp::extract(Serializer& serializer)
{
    serializer.extract(seconds);
    
    serializer.extract(useconds);
    
}

void GpsTimestamp::insert(Serializer& serializer) const
{
    serializer.insert(tow);
    
    serializer.insert(week_number);
    
    serializer.insert(valid_flags);
    
}
void GpsTimestamp::extract(Serializer& serializer)
{
    serializer.extract(tow);
    
    serializer.extract(week_number);
    
    serializer.extract(valid_flags);
    
}

void TemperatureAbs::insert(Serializer& serializer) const
{
    serializer.insert(min_temp);
    
    serializer.insert(max_temp);
    
    serializer.insert(mean_temp);
    
}
void TemperatureAbs::extract(Serializer& serializer)
{
    serializer.extract(min_temp);
    
    serializer.extract(max_temp);
    
    serializer.extract(mean_temp);
    
}

void UpVector::insert(Serializer& serializer) const
{
    serializer.insert(up);
    
}
void UpVector::extract(Serializer& serializer)
{
    serializer.extract(up);
    
}

void NorthVector::insert(Serializer& serializer) const
{
    serializer.insert(north);
    
}
void NorthVector::extract(Serializer& serializer)
{
    serializer.extract(north);
    
}

void OverrangeStatus::insert(Serializer& serializer) const
{
    serializer.insert(status);
    
}
void OverrangeStatus::extract(Serializer& serializer)
{
    serializer.extract(status);
    
}

void OdometerData::insert(Serializer& serializer) const
{
    serializer.insert(speed);
    
    serializer.insert(uncertainty);
    
    serializer.insert(valid_flags);
    
}
void OdometerData::extract(Serializer& serializer)
{
    serializer.extract(speed);
    
    serializer.extract(uncertainty);
    
    serializer.extract(valid_flags);
    
}


} // namespace data_sensor
} // namespace mip

