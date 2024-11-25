
#include "data_gnss.hpp"

#include <mip/mip_serialization.hpp>
#include <mip/mip_interface.h>

#include <assert.h>


namespace mip {
namespace C {
struct mip_interface;
} // namespace C

namespace data_gnss {

using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void PosLlh::insert(Serializer& serializer) const
{
    serializer.insert(latitude);
    
    serializer.insert(longitude);
    
    serializer.insert(ellipsoid_height);
    
    serializer.insert(msl_height);
    
    serializer.insert(horizontal_accuracy);
    
    serializer.insert(vertical_accuracy);
    
    serializer.insert(valid_flags);
    
}
void PosLlh::extract(Serializer& serializer)
{
    serializer.extract(latitude);
    
    serializer.extract(longitude);
    
    serializer.extract(ellipsoid_height);
    
    serializer.extract(msl_height);
    
    serializer.extract(horizontal_accuracy);
    
    serializer.extract(vertical_accuracy);
    
    serializer.extract(valid_flags);
    
}

void PosEcef::insert(Serializer& serializer) const
{
    serializer.insert(x);
    
    serializer.insert(x_accuracy);
    
    serializer.insert(valid_flags);
    
}
void PosEcef::extract(Serializer& serializer)
{
    serializer.extract(x);
    
    serializer.extract(x_accuracy);
    
    serializer.extract(valid_flags);
    
}

void VelNed::insert(Serializer& serializer) const
{
    serializer.insert(v);
    
    serializer.insert(speed);
    
    serializer.insert(ground_speed);
    
    serializer.insert(heading);
    
    serializer.insert(speed_accuracy);
    
    serializer.insert(heading_accuracy);
    
    serializer.insert(valid_flags);
    
}
void VelNed::extract(Serializer& serializer)
{
    serializer.extract(v);
    
    serializer.extract(speed);
    
    serializer.extract(ground_speed);
    
    serializer.extract(heading);
    
    serializer.extract(speed_accuracy);
    
    serializer.extract(heading_accuracy);
    
    serializer.extract(valid_flags);
    
}

void VelEcef::insert(Serializer& serializer) const
{
    serializer.insert(v);
    
    serializer.insert(v_accuracy);
    
    serializer.insert(valid_flags);
    
}
void VelEcef::extract(Serializer& serializer)
{
    serializer.extract(v);
    
    serializer.extract(v_accuracy);
    
    serializer.extract(valid_flags);
    
}

void Dop::insert(Serializer& serializer) const
{
    serializer.insert(gdop);
    
    serializer.insert(pdop);
    
    serializer.insert(hdop);
    
    serializer.insert(vdop);
    
    serializer.insert(tdop);
    
    serializer.insert(ndop);
    
    serializer.insert(edop);
    
    serializer.insert(valid_flags);
    
}
void Dop::extract(Serializer& serializer)
{
    serializer.extract(gdop);
    
    serializer.extract(pdop);
    
    serializer.extract(hdop);
    
    serializer.extract(vdop);
    
    serializer.extract(tdop);
    
    serializer.extract(ndop);
    
    serializer.extract(edop);
    
    serializer.extract(valid_flags);
    
}

void UtcTime::insert(Serializer& serializer) const
{
    serializer.insert(year);
    
    serializer.insert(month);
    
    serializer.insert(day);
    
    serializer.insert(hour);
    
    serializer.insert(min);
    
    serializer.insert(sec);
    
    serializer.insert(msec);
    
    serializer.insert(valid_flags);
    
}
void UtcTime::extract(Serializer& serializer)
{
    serializer.extract(year);
    
    serializer.extract(month);
    
    serializer.extract(day);
    
    serializer.extract(hour);
    
    serializer.extract(min);
    
    serializer.extract(sec);
    
    serializer.extract(msec);
    
    serializer.extract(valid_flags);
    
}

void GpsTime::insert(Serializer& serializer) const
{
    serializer.insert(tow);
    
    serializer.insert(week_number);
    
    serializer.insert(valid_flags);
    
}
void GpsTime::extract(Serializer& serializer)
{
    serializer.extract(tow);
    
    serializer.extract(week_number);
    
    serializer.extract(valid_flags);
    
}

void ClockInfo::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
    serializer.insert(drift);
    
    serializer.insert(accuracy_estimate);
    
    serializer.insert(valid_flags);
    
}
void ClockInfo::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
    serializer.extract(drift);
    
    serializer.extract(accuracy_estimate);
    
    serializer.extract(valid_flags);
    
}

void FixInfo::insert(Serializer& serializer) const
{
    serializer.insert(fix_type);
    
    serializer.insert(num_sv);
    
    serializer.insert(fix_flags);
    
    serializer.insert(valid_flags);
    
}
void FixInfo::extract(Serializer& serializer)
{
    serializer.extract(fix_type);
    
    serializer.extract(num_sv);
    
    serializer.extract(fix_flags);
    
    serializer.extract(valid_flags);
    
}

void SvInfo::insert(Serializer& serializer) const
{
    serializer.insert(channel);
    
    serializer.insert(sv_id);
    
    serializer.insert(carrier_noise_ratio);
    
    serializer.insert(azimuth);
    
    serializer.insert(elevation);
    
    serializer.insert(sv_flags);
    
    serializer.insert(valid_flags);
    
}
void SvInfo::extract(Serializer& serializer)
{
    serializer.extract(channel);
    
    serializer.extract(sv_id);
    
    serializer.extract(carrier_noise_ratio);
    
    serializer.extract(azimuth);
    
    serializer.extract(elevation);
    
    serializer.extract(sv_flags);
    
    serializer.extract(valid_flags);
    
}

void HwStatus::insert(Serializer& serializer) const
{
    serializer.insert(receiver_state);
    
    serializer.insert(antenna_state);
    
    serializer.insert(antenna_power);
    
    serializer.insert(valid_flags);
    
}
void HwStatus::extract(Serializer& serializer)
{
    serializer.extract(receiver_state);
    
    serializer.extract(antenna_state);
    
    serializer.extract(antenna_power);
    
    serializer.extract(valid_flags);
    
}

void DgpsInfo::insert(Serializer& serializer) const
{
    serializer.insert(sv_id);
    
    serializer.insert(age);
    
    serializer.insert(range_correction);
    
    serializer.insert(range_rate_correction);
    
    serializer.insert(valid_flags);
    
}
void DgpsInfo::extract(Serializer& serializer)
{
    serializer.extract(sv_id);
    
    serializer.extract(age);
    
    serializer.extract(range_correction);
    
    serializer.extract(range_rate_correction);
    
    serializer.extract(valid_flags);
    
}

void DgpsChannel::insert(Serializer& serializer) const
{
    serializer.insert(sv_id);
    
    serializer.insert(age);
    
    serializer.insert(range_correction);
    
    serializer.insert(range_rate_correction);
    
    serializer.insert(valid_flags);
    
}
void DgpsChannel::extract(Serializer& serializer)
{
    serializer.extract(sv_id);
    
    serializer.extract(age);
    
    serializer.extract(range_correction);
    
    serializer.extract(range_rate_correction);
    
    serializer.extract(valid_flags);
    
}

void ClockInfo2::insert(Serializer& serializer) const
{
    serializer.insert(bias);
    
    serializer.insert(drift);
    
    serializer.insert(bias_accuracy_estimate);
    
    serializer.insert(drift_accuracy_estimate);
    
    serializer.insert(valid_flags);
    
}
void ClockInfo2::extract(Serializer& serializer)
{
    serializer.extract(bias);
    
    serializer.extract(drift);
    
    serializer.extract(bias_accuracy_estimate);
    
    serializer.extract(drift_accuracy_estimate);
    
    serializer.extract(valid_flags);
    
}

void GpsLeapSeconds::insert(Serializer& serializer) const
{
    serializer.insert(leap_seconds);
    
    serializer.insert(valid_flags);
    
}
void GpsLeapSeconds::extract(Serializer& serializer)
{
    serializer.extract(leap_seconds);
    
    serializer.extract(valid_flags);
    
}

void SbasInfo::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(sbas_system);
    
    serializer.insert(sbas_id);
    
    serializer.insert(count);
    
    serializer.insert(sbas_status);
    
    serializer.insert(valid_flags);
    
}
void SbasInfo::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(sbas_system);
    
    serializer.extract(sbas_id);
    
    serializer.extract(count);
    
    serializer.extract(sbas_status);
    
    serializer.extract(valid_flags);
    
}

void SbasCorrection::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(gnss_id);
    
    serializer.insert(sv_id);
    
    serializer.insert(udrei);
    
    serializer.insert(pseudorange_correction);
    
    serializer.insert(iono_correction);
    
    serializer.insert(valid_flags);
    
}
void SbasCorrection::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(gnss_id);
    
    serializer.extract(sv_id);
    
    serializer.extract(udrei);
    
    serializer.extract(pseudorange_correction);
    
    serializer.extract(iono_correction);
    
    serializer.extract(valid_flags);
    
}

void RfErrorDetection::insert(Serializer& serializer) const
{
    serializer.insert(rf_band);
    
    serializer.insert(jamming_state);
    
    serializer.insert(spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(reserved[i]);
    
    serializer.insert(valid_flags);
    
}
void RfErrorDetection::extract(Serializer& serializer)
{
    serializer.extract(rf_band);
    
    serializer.extract(jamming_state);
    
    serializer.extract(spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(reserved[i]);
    
    serializer.extract(valid_flags);
    
}

void BaseStationInfo::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(ecef_pos);
    
    serializer.insert(height);
    
    serializer.insert(station_id);
    
    serializer.insert(indicators);
    
    serializer.insert(valid_flags);
    
}
void BaseStationInfo::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(ecef_pos);
    
    serializer.extract(height);
    
    serializer.extract(station_id);
    
    serializer.extract(indicators);
    
    serializer.extract(valid_flags);
    
}

void RtkCorrectionsStatus::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(epoch_status);
    
    serializer.insert(dongle_status);
    
    serializer.insert(gps_correction_latency);
    
    serializer.insert(glonass_correction_latency);
    
    serializer.insert(galileo_correction_latency);
    
    serializer.insert(beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(reserved[i]);
    
    serializer.insert(valid_flags);
    
}
void RtkCorrectionsStatus::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(epoch_status);
    
    serializer.extract(dongle_status);
    
    serializer.extract(gps_correction_latency);
    
    serializer.extract(glonass_correction_latency);
    
    serializer.extract(galileo_correction_latency);
    
    serializer.extract(beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(reserved[i]);
    
    serializer.extract(valid_flags);
    
}

void SatelliteStatus::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(gnss_id);
    
    serializer.insert(satellite_id);
    
    serializer.insert(elevation);
    
    serializer.insert(azimuth);
    
    serializer.insert(health);
    
    serializer.insert(valid_flags);
    
}
void SatelliteStatus::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(gnss_id);
    
    serializer.extract(satellite_id);
    
    serializer.extract(elevation);
    
    serializer.extract(azimuth);
    
    serializer.extract(health);
    
    serializer.extract(valid_flags);
    
}

void Raw::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(receiver_id);
    
    serializer.insert(tracking_channel);
    
    serializer.insert(gnss_id);
    
    serializer.insert(satellite_id);
    
    serializer.insert(signal_id);
    
    serializer.insert(signal_strength);
    
    serializer.insert(quality);
    
    serializer.insert(pseudorange);
    
    serializer.insert(carrier_phase);
    
    serializer.insert(doppler);
    
    serializer.insert(range_uncert);
    
    serializer.insert(phase_uncert);
    
    serializer.insert(doppler_uncert);
    
    serializer.insert(lock_time);
    
    serializer.insert(valid_flags);
    
}
void Raw::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(receiver_id);
    
    serializer.extract(tracking_channel);
    
    serializer.extract(gnss_id);
    
    serializer.extract(satellite_id);
    
    serializer.extract(signal_id);
    
    serializer.extract(signal_strength);
    
    serializer.extract(quality);
    
    serializer.extract(pseudorange);
    
    serializer.extract(carrier_phase);
    
    serializer.extract(doppler);
    
    serializer.extract(range_uncert);
    
    serializer.extract(phase_uncert);
    
    serializer.extract(doppler_uncert);
    
    serializer.extract(lock_time);
    
    serializer.extract(valid_flags);
    
}

void GpsEphemeris::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(satellite_id);
    
    serializer.insert(health);
    
    serializer.insert(iodc);
    
    serializer.insert(iode);
    
    serializer.insert(t_oc);
    
    serializer.insert(af0);
    
    serializer.insert(af1);
    
    serializer.insert(af2);
    
    serializer.insert(t_gd);
    
    serializer.insert(ISC_L1CA);
    
    serializer.insert(ISC_L2C);
    
    serializer.insert(t_oe);
    
    serializer.insert(a);
    
    serializer.insert(a_dot);
    
    serializer.insert(mean_anomaly);
    
    serializer.insert(delta_mean_motion);
    
    serializer.insert(delta_mean_motion_dot);
    
    serializer.insert(eccentricity);
    
    serializer.insert(argument_of_perigee);
    
    serializer.insert(omega);
    
    serializer.insert(omega_dot);
    
    serializer.insert(inclination);
    
    serializer.insert(inclination_dot);
    
    serializer.insert(c_ic);
    
    serializer.insert(c_is);
    
    serializer.insert(c_uc);
    
    serializer.insert(c_us);
    
    serializer.insert(c_rc);
    
    serializer.insert(c_rs);
    
    serializer.insert(valid_flags);
    
}
void GpsEphemeris::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(satellite_id);
    
    serializer.extract(health);
    
    serializer.extract(iodc);
    
    serializer.extract(iode);
    
    serializer.extract(t_oc);
    
    serializer.extract(af0);
    
    serializer.extract(af1);
    
    serializer.extract(af2);
    
    serializer.extract(t_gd);
    
    serializer.extract(ISC_L1CA);
    
    serializer.extract(ISC_L2C);
    
    serializer.extract(t_oe);
    
    serializer.extract(a);
    
    serializer.extract(a_dot);
    
    serializer.extract(mean_anomaly);
    
    serializer.extract(delta_mean_motion);
    
    serializer.extract(delta_mean_motion_dot);
    
    serializer.extract(eccentricity);
    
    serializer.extract(argument_of_perigee);
    
    serializer.extract(omega);
    
    serializer.extract(omega_dot);
    
    serializer.extract(inclination);
    
    serializer.extract(inclination_dot);
    
    serializer.extract(c_ic);
    
    serializer.extract(c_is);
    
    serializer.extract(c_uc);
    
    serializer.extract(c_us);
    
    serializer.extract(c_rc);
    
    serializer.extract(c_rs);
    
    serializer.extract(valid_flags);
    
}

void GalileoEphemeris::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(satellite_id);
    
    serializer.insert(health);
    
    serializer.insert(iodc);
    
    serializer.insert(iode);
    
    serializer.insert(t_oc);
    
    serializer.insert(af0);
    
    serializer.insert(af1);
    
    serializer.insert(af2);
    
    serializer.insert(t_gd);
    
    serializer.insert(ISC_L1CA);
    
    serializer.insert(ISC_L2C);
    
    serializer.insert(t_oe);
    
    serializer.insert(a);
    
    serializer.insert(a_dot);
    
    serializer.insert(mean_anomaly);
    
    serializer.insert(delta_mean_motion);
    
    serializer.insert(delta_mean_motion_dot);
    
    serializer.insert(eccentricity);
    
    serializer.insert(argument_of_perigee);
    
    serializer.insert(omega);
    
    serializer.insert(omega_dot);
    
    serializer.insert(inclination);
    
    serializer.insert(inclination_dot);
    
    serializer.insert(c_ic);
    
    serializer.insert(c_is);
    
    serializer.insert(c_uc);
    
    serializer.insert(c_us);
    
    serializer.insert(c_rc);
    
    serializer.insert(c_rs);
    
    serializer.insert(valid_flags);
    
}
void GalileoEphemeris::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(satellite_id);
    
    serializer.extract(health);
    
    serializer.extract(iodc);
    
    serializer.extract(iode);
    
    serializer.extract(t_oc);
    
    serializer.extract(af0);
    
    serializer.extract(af1);
    
    serializer.extract(af2);
    
    serializer.extract(t_gd);
    
    serializer.extract(ISC_L1CA);
    
    serializer.extract(ISC_L2C);
    
    serializer.extract(t_oe);
    
    serializer.extract(a);
    
    serializer.extract(a_dot);
    
    serializer.extract(mean_anomaly);
    
    serializer.extract(delta_mean_motion);
    
    serializer.extract(delta_mean_motion_dot);
    
    serializer.extract(eccentricity);
    
    serializer.extract(argument_of_perigee);
    
    serializer.extract(omega);
    
    serializer.extract(omega_dot);
    
    serializer.extract(inclination);
    
    serializer.extract(inclination_dot);
    
    serializer.extract(c_ic);
    
    serializer.extract(c_is);
    
    serializer.extract(c_uc);
    
    serializer.extract(c_us);
    
    serializer.extract(c_rc);
    
    serializer.extract(c_rs);
    
    serializer.extract(valid_flags);
    
}

void GloEphemeris::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(satellite_id);
    
    serializer.insert(freq_number);
    
    serializer.insert(tk);
    
    serializer.insert(tb);
    
    serializer.insert(sat_type);
    
    serializer.insert(gamma);
    
    serializer.insert(tau_n);
    
    serializer.insert(x);
    
    serializer.insert(v);
    
    serializer.insert(a);
    
    serializer.insert(health);
    
    serializer.insert(P);
    
    serializer.insert(NT);
    
    serializer.insert(delta_tau_n);
    
    serializer.insert(Ft);
    
    serializer.insert(En);
    
    serializer.insert(P1);
    
    serializer.insert(P2);
    
    serializer.insert(P3);
    
    serializer.insert(P4);
    
    serializer.insert(valid_flags);
    
}
void GloEphemeris::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(satellite_id);
    
    serializer.extract(freq_number);
    
    serializer.extract(tk);
    
    serializer.extract(tb);
    
    serializer.extract(sat_type);
    
    serializer.extract(gamma);
    
    serializer.extract(tau_n);
    
    serializer.extract(x);
    
    serializer.extract(v);
    
    serializer.extract(a);
    
    serializer.extract(health);
    
    serializer.extract(P);
    
    serializer.extract(NT);
    
    serializer.extract(delta_tau_n);
    
    serializer.extract(Ft);
    
    serializer.extract(En);
    
    serializer.extract(P1);
    
    serializer.extract(P2);
    
    serializer.extract(P3);
    
    serializer.extract(P4);
    
    serializer.extract(valid_flags);
    
}

void BeidouEphemeris::insert(Serializer& serializer) const
{
    serializer.insert(index);
    
    serializer.insert(count);
    
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(satellite_id);
    
    serializer.insert(health);
    
    serializer.insert(iodc);
    
    serializer.insert(iode);
    
    serializer.insert(t_oc);
    
    serializer.insert(af0);
    
    serializer.insert(af1);
    
    serializer.insert(af2);
    
    serializer.insert(t_gd);
    
    serializer.insert(ISC_L1CA);
    
    serializer.insert(ISC_L2C);
    
    serializer.insert(t_oe);
    
    serializer.insert(a);
    
    serializer.insert(a_dot);
    
    serializer.insert(mean_anomaly);
    
    serializer.insert(delta_mean_motion);
    
    serializer.insert(delta_mean_motion_dot);
    
    serializer.insert(eccentricity);
    
    serializer.insert(argument_of_perigee);
    
    serializer.insert(omega);
    
    serializer.insert(omega_dot);
    
    serializer.insert(inclination);
    
    serializer.insert(inclination_dot);
    
    serializer.insert(c_ic);
    
    serializer.insert(c_is);
    
    serializer.insert(c_uc);
    
    serializer.insert(c_us);
    
    serializer.insert(c_rc);
    
    serializer.insert(c_rs);
    
    serializer.insert(valid_flags);
    
}
void BeidouEphemeris::extract(Serializer& serializer)
{
    serializer.extract(index);
    
    serializer.extract(count);
    
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(satellite_id);
    
    serializer.extract(health);
    
    serializer.extract(iodc);
    
    serializer.extract(iode);
    
    serializer.extract(t_oc);
    
    serializer.extract(af0);
    
    serializer.extract(af1);
    
    serializer.extract(af2);
    
    serializer.extract(t_gd);
    
    serializer.extract(ISC_L1CA);
    
    serializer.extract(ISC_L2C);
    
    serializer.extract(t_oe);
    
    serializer.extract(a);
    
    serializer.extract(a_dot);
    
    serializer.extract(mean_anomaly);
    
    serializer.extract(delta_mean_motion);
    
    serializer.extract(delta_mean_motion_dot);
    
    serializer.extract(eccentricity);
    
    serializer.extract(argument_of_perigee);
    
    serializer.extract(omega);
    
    serializer.extract(omega_dot);
    
    serializer.extract(inclination);
    
    serializer.extract(inclination_dot);
    
    serializer.extract(c_ic);
    
    serializer.extract(c_is);
    
    serializer.extract(c_uc);
    
    serializer.extract(c_us);
    
    serializer.extract(c_rc);
    
    serializer.extract(c_rs);
    
    serializer.extract(valid_flags);
    
}

void GpsIonoCorr::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(beta[i]);
    
    serializer.insert(valid_flags);
    
}
void GpsIonoCorr::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(beta[i]);
    
    serializer.extract(valid_flags);
    
}

void GalileoIonoCorr::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    serializer.insert(alpha);
    
    serializer.insert(disturbance_flags);
    
    serializer.insert(valid_flags);
    
}
void GalileoIonoCorr::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    serializer.extract(alpha);
    
    serializer.extract(disturbance_flags);
    
    serializer.extract(valid_flags);
    
}

void BeidouIonoCorr::insert(Serializer& serializer) const
{
    serializer.insert(time_of_week);
    
    serializer.insert(week_number);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.insert(beta[i]);
    
    for(unsigned int i=0; i < 9; i++)
        serializer.insert(alpha_corr[i]);
    
    serializer.insert(valid_flags);
    
}
void BeidouIonoCorr::extract(Serializer& serializer)
{
    serializer.extract(time_of_week);
    
    serializer.extract(week_number);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        serializer.extract(beta[i]);
    
    for(unsigned int i=0; i < 9; i++)
        serializer.extract(alpha_corr[i]);
    
    serializer.extract(valid_flags);
    
}


} // namespace data_gnss
} // namespace mip

