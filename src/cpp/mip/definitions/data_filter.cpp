
#include "data_filter.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_filter {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void PositionLlh::insert(Serializer& serializer) const
{
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(ellipsoid_height);
    
    serializer.insert(valid_flags);
    
}
void PositionLlh::extract(Serializer& serializer)
{
    serializer.extract(latitude);
    
    serializer.extract(longitude);
    
    serializer.extract(ellipsoid_height);
    
    serializer.extract(valid_flags);
    
}

void VelocityNed::insert(Serializer& serializer) const
{
    serializer.insert(north);
    
    serializer.insert(east);
    
    serializer.insert(down);
    
    serializer.insert(valid_flags);
    
}
void VelocityNed::extract(Serializer& serializer)
{
    serializer.extract(north);
    
    serializer.extract(east);
    
    serializer.extract(down);
    
    serializer.extract(valid_flags);
    
}

void AttitudeQuaternion::insert(Serializer& serializer) const
{
    serializer.insert(q);
    
    serializer.insert(valid_flags);
    
}
void AttitudeQuaternion::extract(Serializer& serializer)
{
    serializer.extract(q);
    
    serializer.extract(valid_flags);
    
}

void AttitudeDcm::insert(Serializer& serializer) const
{
    serializer.insert(dcm);
    
    serializer.insert(valid_flags);
    
}
void AttitudeDcm::extract(Serializer& serializer)
{
    serializer.extract(dcm);
    
    serializer.extract(valid_flags);
    
}

void EulerAngles::insert(Serializer& serializer) const
{
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
    serializer.insert(valid_flags);
    
}
void EulerAngles::extract(Serializer& serializer)
{
    serializer.extract(roll);
    
    serializer.extract(pitch);
    
    serializer.extract(yaw);
    
    serializer.extract(valid_flags);
    
}

void GyroBias::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
    serializer.insert(valid_flags);
    
}
void GyroBias::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
    serializer.extract(valid_flags);
    
}

void AccelBias::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
    serializer.insert(valid_flags);
    
}
void AccelBias::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
    serializer.extract(valid_flags);
    
}

void PositionLlhUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(north);
    
    serializer.insert(east);
    
    serializer.insert(down);
    
    serializer.insert(valid_flags);
    
}
void PositionLlhUncertainty::extract(Serializer& serializer)
{
    serializer.extract(north);
    
    serializer.extract(east);
    
    serializer.extract(down);
    
    serializer.extract(valid_flags);
    
}

void VelocityNedUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(north);
    
    serializer.insert(east);
    
    serializer.insert(down);
    
    serializer.insert(valid_flags);
    
}
void VelocityNedUncertainty::extract(Serializer& serializer)
{
    serializer.extract(north);
    
    serializer.extract(east);
    
    serializer.extract(down);
    
    serializer.extract(valid_flags);
    
}

void EulerAnglesUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(roll);
    
    serializer.insert(pitch);
    
    serializer.insert(yaw);
    
    serializer.insert(valid_flags);
    
}
void EulerAnglesUncertainty::extract(Serializer& serializer)
{
    serializer.extract(roll);
    
    serializer.extract(pitch);
    
    serializer.extract(yaw);
    
    serializer.extract(valid_flags);
    
}

void GyroBiasUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(bias_uncert);
    
    serializer.insert(valid_flags);
    
}
void GyroBiasUncertainty::extract(Serializer& serializer)
{
    serializer.extract(bias_uncert);
    
    serializer.extract(valid_flags);
    
}

void AccelBiasUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(bias_uncert);
    
    serializer.insert(valid_flags);
    
}
void AccelBiasUncertainty::extract(Serializer& serializer)
{
    serializer.extract(bias_uncert);
    
    serializer.extract(valid_flags);
    
}

void Timestamp::insert(Serializer& serializer) const
{
    serializer.insert(tow);
    
    serializer.insert(week_number);
    
    serializer.insert(valid_flags);
    
}
void Timestamp::extract(Serializer& serializer)
{
    serializer.extract(tow);
    
    serializer.extract(week_number);
    
    serializer.extract(valid_flags);
    
}

void Status::insert(Serializer& serializer) const
{
    serializer.insert(filter_state);
    
    serializer.insert(dynamics_mode);
    
    serializer.insert(status_flags);
    
}
void Status::extract(Serializer& serializer)
{
    serializer.extract(filter_state);
    
    serializer.extract(dynamics_mode);
    
    serializer.extract(status_flags);
    
}

void LinearAccel::insert(Serializer& serializer) const
{
    serializer.insert(accel);
    
    serializer.insert(valid_flags);
    
}
void LinearAccel::extract(Serializer& serializer)
{
    serializer.extract(accel);
    
    serializer.extract(valid_flags);
    
}

void GravityVector::insert(Serializer& serializer) const
{
    serializer.insert(gravity);
    
    serializer.insert(valid_flags);
    
}
void GravityVector::extract(Serializer& serializer)
{
    serializer.extract(gravity);
    
    serializer.extract(valid_flags);
    
}

void CompAccel::insert(Serializer& serializer) const
{
    serializer.insert(accel);
    
    serializer.insert(valid_flags);
    
}
void CompAccel::extract(Serializer& serializer)
{
    serializer.extract(accel);
    
    serializer.extract(valid_flags);
    
}

void CompAngularRate::insert(Serializer& serializer) const
{
    serializer.insert(gyro);
    
    serializer.insert(valid_flags);
    
}
void CompAngularRate::extract(Serializer& serializer)
{
    serializer.extract(gyro);
    
    serializer.extract(valid_flags);
    
}

void QuaternionAttitudeUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(q);
    
    serializer.insert(valid_flags);
    
}
void QuaternionAttitudeUncertainty::extract(Serializer& serializer)
{
    serializer.extract(q);
    
    serializer.extract(valid_flags);
    
}

void Wgs84GravityMag::insert(Serializer& serializer) const
{
    serializer.insert(magnitude);
    
    serializer.insert(valid_flags);
    
}
void Wgs84GravityMag::extract(Serializer& serializer)
{
    serializer.extract(magnitude);
    
    serializer.extract(valid_flags);
    
}

void HeadingUpdateState::insert(Serializer& serializer) const
{
    serializer.insert(heading);
    
    serializer.insert(heading_1sigma);
    
    serializer.insert(source);
    
    serializer.insert(valid_flags);
    
}
void HeadingUpdateState::extract(Serializer& serializer)
{
    serializer.extract(heading);
    
    serializer.extract(heading_1sigma);
    
    serializer.extract(source);
    
    serializer.extract(valid_flags);
    
}

void MagneticModel::insert(Serializer& serializer) const
{
    serializer.insert(intensity_north);
    
    serializer.insert(intensity_east);
    
    serializer.insert(intensity_down);
    
    serializer.insert(inclination);
    
    serializer.insert(declination);
    
    serializer.insert(valid_flags);
    
}
void MagneticModel::extract(Serializer& serializer)
{
    serializer.extract(intensity_north);
    
    serializer.extract(intensity_east);
    
    serializer.extract(intensity_down);
    
    serializer.extract(inclination);
    
    serializer.extract(declination);
    
    serializer.extract(valid_flags);
    
}

void AccelScaleFactor::insert(Serializer& serializer) const
{
    serializer.insert(scale_factor);
    
    serializer.insert(valid_flags);
    
}
void AccelScaleFactor::extract(Serializer& serializer)
{
    serializer.extract(scale_factor);
    
    serializer.extract(valid_flags);
    
}

void AccelScaleFactorUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(scale_factor_uncert);
    
    serializer.insert(valid_flags);
    
}
void AccelScaleFactorUncertainty::extract(Serializer& serializer)
{
    serializer.extract(scale_factor_uncert);
    
    serializer.extract(valid_flags);
    
}

void GyroScaleFactor::insert(Serializer& serializer) const
{
    serializer.insert(scale_factor);
    
    serializer.insert(valid_flags);
    
}
void GyroScaleFactor::extract(Serializer& serializer)
{
    serializer.extract(scale_factor);
    
    serializer.extract(valid_flags);
    
}

void GyroScaleFactorUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(scale_factor_uncert);
    
    serializer.insert(valid_flags);
    
}
void GyroScaleFactorUncertainty::extract(Serializer& serializer)
{
    serializer.extract(scale_factor_uncert);
    
    serializer.extract(valid_flags);
    
}

void MagBias::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
    serializer.insert(valid_flags);
    
}
void MagBias::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
    serializer.extract(valid_flags);
    
}

void MagBiasUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(bias_uncert);
    
    serializer.insert(valid_flags);
    
}
void MagBiasUncertainty::extract(Serializer& serializer)
{
    serializer.extract(bias_uncert);
    
    serializer.extract(valid_flags);
    
}

void StandardAtmosphere::insert(Serializer& serializer) const
{
    serializer.insert(geometric_altitude);
    
    serializer.insert(geopotential_altitude);
    
    serializer.insert(standard_temperature);
    
    serializer.insert(standard_pressure);
    
    serializer.insert(standard_density);
    
    serializer.insert(valid_flags);
    
}
void StandardAtmosphere::extract(Serializer& serializer)
{
    serializer.extract(geometric_altitude);
    
    serializer.extract(geopotential_altitude);
    
    serializer.extract(standard_temperature);
    
    serializer.extract(standard_pressure);
    
    serializer.extract(standard_density);
    
    serializer.extract(valid_flags);
    
}

void PressureAltitude::insert(Serializer& serializer) const
{
    serializer.insert(pressure_altitude);
    
    serializer.insert(valid_flags);
    
}
void PressureAltitude::extract(Serializer& serializer)
{
    serializer.extract(pressure_altitude);
    
    serializer.extract(valid_flags);
    
}

void DensityAltitude::insert(Serializer& serializer) const
{
    serializer.insert(density_altitude);
    
    serializer.insert(valid_flags);
    
}
void DensityAltitude::extract(Serializer& serializer)
{
    serializer.extract(density_altitude);
    
    serializer.extract(valid_flags);
    
}

void AntennaOffsetCorrection::insert(Serializer& serializer) const
{
    serializer.insert(offset);
    
    serializer.insert(valid_flags);
    
}
void AntennaOffsetCorrection::extract(Serializer& serializer)
{
    serializer.extract(offset);
    
    serializer.extract(valid_flags);
    
}

void AntennaOffsetCorrectionUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(offset_uncert);
    
    serializer.insert(valid_flags);
    
}
void AntennaOffsetCorrectionUncertainty::extract(Serializer& serializer)
{
    serializer.extract(offset_uncert);
    
    serializer.extract(valid_flags);
    
}

void MultiAntennaOffsetCorrection::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(offset);
    
    serializer.insert(valid_flags);
    
}
void MultiAntennaOffsetCorrection::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(offset);
    
    serializer.extract(valid_flags);
    
}

void MultiAntennaOffsetCorrectionUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(offset_uncert);
    
    serializer.insert(valid_flags);
    
}
void MultiAntennaOffsetCorrectionUncertainty::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(offset_uncert);
    
    serializer.extract(valid_flags);
    
}

void MagnetometerOffset::insert(Serializer& serializer) const
{
    serializer.insert(hard_iron);
    
    serializer.insert(valid_flags);
    
}
void MagnetometerOffset::extract(Serializer& serializer)
{
    serializer.extract(hard_iron);
    
    serializer.extract(valid_flags);
    
}

void MagnetometerMatrix::insert(Serializer& serializer) const
{
    serializer.insert(soft_iron);
    
    serializer.insert(valid_flags);
    
}
void MagnetometerMatrix::extract(Serializer& serializer)
{
    serializer.extract(soft_iron);
    
    serializer.extract(valid_flags);
    
}

void MagnetometerOffsetUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(hard_iron_uncertainty);
    
    serializer.insert(valid_flags);
    
}
void MagnetometerOffsetUncertainty::extract(Serializer& serializer)
{
    serializer.extract(hard_iron_uncertainty);
    
    serializer.extract(valid_flags);
    
}

void MagnetometerMatrixUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(soft_iron_uncertainty);
    
    serializer.insert(valid_flags);
    
}
void MagnetometerMatrixUncertainty::extract(Serializer& serializer)
{
    serializer.extract(soft_iron_uncertainty);
    
    serializer.extract(valid_flags);
    
}

void MagnetometerCovarianceMatrix::insert(Serializer& serializer) const
{
    serializer.insert(covariance);
    
    serializer.insert(valid_flags);
    
}
void MagnetometerCovarianceMatrix::extract(Serializer& serializer)
{
    serializer.extract(covariance);
    
    serializer.extract(valid_flags);
    
}

void MagnetometerResidualVector::insert(Serializer& serializer) const
{
    serializer.insert(residual);
    
    serializer.insert(valid_flags);
    
}
void MagnetometerResidualVector::extract(Serializer& serializer)
{
    serializer.extract(residual);
    
    serializer.extract(valid_flags);
    
}

void ClockCorrection::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(bias);
    
    serializer.insert(bias_drift);
    
    serializer.insert(valid_flags);
    
}
void ClockCorrection::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(bias);
    
    serializer.extract(bias_drift);
    
    serializer.extract(valid_flags);
    
}

void ClockCorrectionUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(bias_uncertainty);
    
    serializer.insert(bias_drift_uncertainty);
    
    serializer.insert(valid_flags);
    
}
void ClockCorrectionUncertainty::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(bias_uncertainty);
    
    serializer.extract(bias_drift_uncertainty);
    
    serializer.extract(valid_flags);
    
}

void GnssPosAidStatus::insert(Serializer& serializer) const
{
    serializer.insert(receiver_id);
    
    serializer.insert(time_of_week);
    
    serializer.insert(status);
    
    for(unsigned int i=0; i < 8; i++)
        serializer.insert(reserved[i]);
    
}
void GnssPosAidStatus::extract(Serializer& serializer)
{
    serializer.extract(receiver_id);
    
    serializer.extract(time_of_week);
    
    serializer.extract(status);
    
    for(unsigned int i=0; i < 8; i++)
        serializer.extract(reserved[i]);
    
}

void GnssAttAidStatus::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(status);
    
    for(unsigned int i=0; i < 8; i++)
        serializer.insert(reserved[i]);
    
}
void GnssAttAidStatus::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(status);
    
    for(unsigned int i=0; i < 8; i++)
        serializer.extract(reserved[i]);
    
}

void HeadAidStatus::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(type);
    
    for(unsigned int i=0; i < 2; i++)
        serializer.insert(reserved[i]);
    
}
void HeadAidStatus::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(type);
    
    for(unsigned int i=0; i < 2; i++)
        serializer.extract(reserved[i]);
    
}

void RelPosNed::insert(Serializer& serializer) const
{
    serializer.insert(relative_position);
    
    serializer.insert(valid_flags);
    
}
void RelPosNed::extract(Serializer& serializer)
{
    serializer.extract(relative_position);
    
    serializer.extract(valid_flags);
    
}

void EcefPos::insert(Serializer& serializer) const
{
    serializer.insert(position_ecef);
    
    serializer.insert(valid_flags);
    
}
void EcefPos::extract(Serializer& serializer)
{
    serializer.extract(position_ecef);
    
    serializer.extract(valid_flags);
    
}

void EcefVel::insert(Serializer& serializer) const
{
    serializer.insert(velocity_ecef);
    
    serializer.insert(valid_flags);
    
}
void EcefVel::extract(Serializer& serializer)
{
    serializer.extract(velocity_ecef);
    
    serializer.extract(valid_flags);
    
}

void EcefPosUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(pos_uncertainty);
    
    serializer.insert(valid_flags);
    
}
void EcefPosUncertainty::extract(Serializer& serializer)
{
    serializer.extract(pos_uncertainty);
    
    serializer.extract(valid_flags);
    
}

void EcefVelUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(vel_uncertainty);
    
    serializer.insert(valid_flags);
    
}
void EcefVelUncertainty::extract(Serializer& serializer)
{
    serializer.extract(vel_uncertainty);
    
    serializer.extract(valid_flags);
    
}

void AidingMeasurementSummary::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(source);
    
    serializer.insert(type);
    
    serializer.insert(indicator);
    
}
void AidingMeasurementSummary::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(source);
    
    serializer.extract(type);
    
    serializer.extract(indicator);
    
}

void OdometerScaleFactorError::insert(Serializer& serializer) const
{
    serializer.insert(scale_factor_error);
    
    serializer.insert(valid_flags);
    
}
void OdometerScaleFactorError::extract(Serializer& serializer)
{
    serializer.extract(scale_factor_error);
    
    serializer.extract(valid_flags);
    
}

void OdometerScaleFactorErrorUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(scale_factor_error_uncertainty);
    
    serializer.insert(valid_flags);
    
}
void OdometerScaleFactorErrorUncertainty::extract(Serializer& serializer)
{
    serializer.extract(scale_factor_error_uncertainty);
    
    serializer.extract(valid_flags);
    
}

void GnssDualAntennaStatus::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(heading);
    
    serializer.insert(heading_unc);
    
    serializer.insert(fix_type);
    
    serializer.insert(status_flags);
    
    serializer.insert(valid_flags);
    
}
void GnssDualAntennaStatus::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(heading);
    
    serializer.extract(heading_unc);
    
    serializer.extract(fix_type);
    
    serializer.extract(status_flags);
    
    serializer.extract(valid_flags);
    
}

void AidingFrameConfigError::insert(Serializer& serializer) const
{
    serializer.insert(frame_id);
    
    serializer.insert(translation);
    
    serializer.insert(attitude);
    
}
void AidingFrameConfigError::extract(Serializer& serializer)
{
    serializer.extract(frame_id);
    
    serializer.extract(translation);
    
    serializer.extract(attitude);
    
}

void AidingFrameConfigErrorUncertainty::insert(Serializer& serializer) const
{
    serializer.insert(frame_id);
    
    serializer.insert(translation_unc);
    
    serializer.insert(attitude_unc);
    
}
void AidingFrameConfigErrorUncertainty::extract(Serializer& serializer)
{
    serializer.extract(frame_id);
    
    serializer.extract(translation_unc);
    
    serializer.extract(attitude_unc);
    
}


} // namespace data_filter
} // namespace mip

