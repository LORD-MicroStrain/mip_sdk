
#include "data_filter.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_filter {

using ::mscl::insert;
using ::mscl::extract;
using namespace ::mscl::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const PositionLlh& self)
{
    insert(serializer, self.latitude);
    insert(serializer, self.longitude);
    insert(serializer, self.ellipsoid_height);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, PositionLlh& self)
{
    extract(serializer, self.latitude);
    extract(serializer, self.longitude);
    extract(serializer, self.ellipsoid_height);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const VelocityNed& self)
{
    insert(serializer, self.north);
    insert(serializer, self.east);
    insert(serializer, self.down);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, VelocityNed& self)
{
    extract(serializer, self.north);
    extract(serializer, self.east);
    extract(serializer, self.down);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AttitudeQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AttitudeQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AttitudeDcm& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AttitudeDcm& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const EulerAngles& self)
{
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, EulerAngles& self)
{
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GyroBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GyroBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AccelBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AccelBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const PositionLlhUncertainty& self)
{
    insert(serializer, self.north);
    insert(serializer, self.east);
    insert(serializer, self.down);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, PositionLlhUncertainty& self)
{
    extract(serializer, self.north);
    extract(serializer, self.east);
    extract(serializer, self.down);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const VelocityNedUncertainty& self)
{
    insert(serializer, self.north);
    insert(serializer, self.east);
    insert(serializer, self.down);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, VelocityNedUncertainty& self)
{
    extract(serializer, self.north);
    extract(serializer, self.east);
    extract(serializer, self.down);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const EulerAnglesUncertainty& self)
{
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, EulerAnglesUncertainty& self)
{
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GyroBiasUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GyroBiasUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AccelBiasUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AccelBiasUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const Timestamp& self)
{
    insert(serializer, self.tow);
    insert(serializer, self.week_number);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, Timestamp& self)
{
    extract(serializer, self.tow);
    extract(serializer, self.week_number);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const Status& self)
{
    insert(serializer, self.filter_state);
    insert(serializer, self.dynamics_mode);
    insert(serializer, self.status_flags);
}

void extract(MipSerializer& serializer, Status& self)
{
    extract(serializer, self.filter_state);
    extract(serializer, self.dynamics_mode);
    extract(serializer, self.status_flags);
}

void insert(MipSerializer& serializer, const LinearAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.accel[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, LinearAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.accel[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GravityVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.gravity[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GravityVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.gravity[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const CompAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.accel[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, CompAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.accel[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const CompAngularRate& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.gyro[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, CompAngularRate& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.gyro[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const QuaternionAttitudeUncertainty& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, QuaternionAttitudeUncertainty& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const Wgs84GravityMag& self)
{
    insert(serializer, self.magnitude);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, Wgs84GravityMag& self)
{
    extract(serializer, self.magnitude);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const HeadingUpdateState& self)
{
    insert(serializer, self.heading);
    insert(serializer, self.heading_1sigma);
    insert(serializer, self.source);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, HeadingUpdateState& self)
{
    extract(serializer, self.heading);
    extract(serializer, self.heading_1sigma);
    extract(serializer, self.source);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagneticModel& self)
{
    insert(serializer, self.intensity_north);
    insert(serializer, self.intensity_east);
    insert(serializer, self.intensity_down);
    insert(serializer, self.inclination);
    insert(serializer, self.declination);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagneticModel& self)
{
    extract(serializer, self.intensity_north);
    extract(serializer, self.intensity_east);
    extract(serializer, self.intensity_down);
    extract(serializer, self.inclination);
    extract(serializer, self.declination);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AccelScaleFactor& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scale_factor[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AccelScaleFactor& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scale_factor[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AccelScaleFactorUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scale_factor_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AccelScaleFactorUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scale_factor_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GyroScaleFactor& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scale_factor[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GyroScaleFactor& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scale_factor[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GyroScaleFactorUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scale_factor_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GyroScaleFactorUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scale_factor_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagBias& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagBiasUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagBiasUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const StandardAtmosphere& self)
{
    insert(serializer, self.geometric_altitude);
    insert(serializer, self.geopotential_altitude);
    insert(serializer, self.standard_temperature);
    insert(serializer, self.standard_pressure);
    insert(serializer, self.standard_density);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, StandardAtmosphere& self)
{
    extract(serializer, self.geometric_altitude);
    extract(serializer, self.geopotential_altitude);
    extract(serializer, self.standard_temperature);
    extract(serializer, self.standard_pressure);
    extract(serializer, self.standard_density);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const PressureAltitude& self)
{
    insert(serializer, self.pressure_altitude);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, PressureAltitude& self)
{
    extract(serializer, self.pressure_altitude);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const DensityAltitude& self)
{
    insert(serializer, self.density_altitude);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, DensityAltitude& self)
{
    extract(serializer, self.density_altitude);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AntennaOffsetCorrection& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AntennaOffsetCorrection& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AntennaOffsetCorrectionUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, AntennaOffsetCorrectionUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MultiAntennaOffsetCorrection& self)
{
    insert(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MultiAntennaOffsetCorrection& self)
{
    extract(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MultiAntennaOffsetCorrectionUncertainty& self)
{
    insert(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset_uncert[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MultiAntennaOffsetCorrectionUncertainty& self)
{
    extract(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset_uncert[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagnetometerOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.hard_iron[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagnetometerOffset& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.hard_iron[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagnetometerMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.soft_iron[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagnetometerMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.soft_iron[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagnetometerOffsetUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.hard_iron_uncertainty[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagnetometerOffsetUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.hard_iron_uncertainty[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagnetometerMatrixUncertainty& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.soft_iron_uncertainty[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagnetometerMatrixUncertainty& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.soft_iron_uncertainty[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagnetometerCovarianceMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.covariance[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagnetometerCovarianceMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.covariance[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const MagnetometerResidualVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.residual[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, MagnetometerResidualVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.residual[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const ClockCorrection& self)
{
    insert(serializer, self.receiver_id);
    insert(serializer, self.bias);
    insert(serializer, self.bias_drift);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, ClockCorrection& self)
{
    extract(serializer, self.receiver_id);
    extract(serializer, self.bias);
    extract(serializer, self.bias_drift);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const ClockCorrectionUncertainty& self)
{
    insert(serializer, self.receiver_id);
    insert(serializer, self.bias_uncertainty);
    insert(serializer, self.bias_drift_uncertainty);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, ClockCorrectionUncertainty& self)
{
    extract(serializer, self.receiver_id);
    extract(serializer, self.bias_uncertainty);
    extract(serializer, self.bias_drift_uncertainty);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GnssPosAidStatus& self)
{
    insert(serializer, self.receiver_id);
    insert(serializer, self.time_of_week);
    insert(serializer, self.status);
    for(unsigned int i=0; i < 8; i++)
        insert(serializer, self.reserved[i]);
}

void extract(MipSerializer& serializer, GnssPosAidStatus& self)
{
    extract(serializer, self.receiver_id);
    extract(serializer, self.time_of_week);
    extract(serializer, self.status);
    for(unsigned int i=0; i < 8; i++)
        extract(serializer, self.reserved[i]);
}

void insert(MipSerializer& serializer, const GnssAttAidStatus& self)
{
    insert(serializer, self.time_of_week);
    insert(serializer, self.status);
    for(unsigned int i=0; i < 8; i++)
        insert(serializer, self.reserved[i]);
}

void extract(MipSerializer& serializer, GnssAttAidStatus& self)
{
    extract(serializer, self.time_of_week);
    extract(serializer, self.status);
    for(unsigned int i=0; i < 8; i++)
        extract(serializer, self.reserved[i]);
}

void insert(MipSerializer& serializer, const HeadAidStatus& self)
{
    insert(serializer, self.time_of_week);
    insert(serializer, self.type);
    for(unsigned int i=0; i < 2; i++)
        insert(serializer, self.reserved[i]);
}

void extract(MipSerializer& serializer, HeadAidStatus& self)
{
    extract(serializer, self.time_of_week);
    extract(serializer, self.type);
    for(unsigned int i=0; i < 2; i++)
        extract(serializer, self.reserved[i]);
}

void insert(MipSerializer& serializer, const RelPosNed& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.relative_position[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, RelPosNed& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.relative_position[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const EcefPos& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.position_ecef[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, EcefPos& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.position_ecef[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const EcefVel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity_ecef[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, EcefVel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity_ecef[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const EcefPosUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.pos_uncertainty[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, EcefPosUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.pos_uncertainty[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const EcefVelUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.vel_uncertainty[i]);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, EcefVelUncertainty& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.vel_uncertainty[i]);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const AidingMeasurementSummary& self)
{
    insert(serializer, self.time_of_week);
    insert(serializer, self.source);
    insert(serializer, self.type);
    insert(serializer, self.indicator);
}

void extract(MipSerializer& serializer, AidingMeasurementSummary& self)
{
    extract(serializer, self.time_of_week);
    extract(serializer, self.source);
    extract(serializer, self.type);
    extract(serializer, self.indicator);
}

void insert(MipSerializer& serializer, const OdometerScaleFactorError& self)
{
    insert(serializer, self.scale_factor_error);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, OdometerScaleFactorError& self)
{
    extract(serializer, self.scale_factor_error);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const OdometerScaleFactorErrorUncertainty& self)
{
    insert(serializer, self.scale_factor_error_uncertainty);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, OdometerScaleFactorErrorUncertainty& self)
{
    extract(serializer, self.scale_factor_error_uncertainty);
    extract(serializer, self.valid_flags);
}

void insert(MipSerializer& serializer, const GnssDualAntennaStatus& self)
{
    insert(serializer, self.time_of_week);
    insert(serializer, self.heading);
    insert(serializer, self.heading_unc);
    insert(serializer, self.fix_type);
    insert(serializer, self.status_flags);
    insert(serializer, self.valid_flags);
}

void extract(MipSerializer& serializer, GnssDualAntennaStatus& self)
{
    extract(serializer, self.time_of_week);
    extract(serializer, self.heading);
    extract(serializer, self.heading_unc);
    extract(serializer, self.fix_type);
    extract(serializer, self.status_flags);
    extract(serializer, self.valid_flags);
}


} // namespace data_filter
} // namespace mscl

