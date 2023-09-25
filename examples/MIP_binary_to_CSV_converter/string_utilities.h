#pragma once

#include <iomanip>
#include <vector>
#include <sstream>
#include <algorithm>
#include <experimental/tuple>

#include "mip/mip_all.hpp"

const std::map<uint8_t , std::string> DESCRIPTOR_SET_NAME_MAP = {
        {mip::data_sensor::DESCRIPTOR_SET, "sensor"},
        {mip::data_filter::DESCRIPTOR_SET, "filter"},
        {mip::data_gnss::DESCRIPTOR_SET, "gnss"},
        {mip::data_gnss::MIP_GNSS1_DATA_DESC_SET, "gnss_1"},
        {mip::data_gnss::MIP_GNSS2_DATA_DESC_SET, "gnss_2"},
        {mip::data_gnss::MIP_GNSS3_DATA_DESC_SET, "gnss_3"},
        {mip::data_gnss::MIP_GNSS4_DATA_DESC_SET, "gnss_4"},
        {mip::data_gnss::MIP_GNSS5_DATA_DESC_SET, "gnss_5"},
};


const std::vector<uint8_t> GNSS_DESCRIPTOR_SETS = {mip::data_gnss::DESCRIPTOR_SET,
                                                   mip::data_gnss::MIP_GNSS1_DATA_DESC_SET,
                                                   mip::data_gnss::MIP_GNSS2_DATA_DESC_SET,
                                                   mip::data_gnss::MIP_GNSS3_DATA_DESC_SET,
                                                   mip::data_gnss::MIP_GNSS4_DATA_DESC_SET,
                                                   mip::data_gnss::MIP_GNSS5_DATA_DESC_SET};


const std::vector<uint8_t> SHARED_FIELD_DESCRIPTORS = {mip::data_shared::DATA_EVENT_SOURCE,
                                                       mip::data_shared::DATA_TICKS,
                                                       mip::data_shared::DATA_DELTA_TICKS,
                                                       mip::data_shared::DATA_GPS_TIME,
                                                       mip::data_shared::DATA_DELTA_TIME,
                                                       mip::data_shared::DATA_REFERENCE_TIME,
                                                       mip::data_shared::DATA_REF_TIME_DELTA,
                                                       mip::data_shared::DATA_EXTERNAL_TIME,
                                                       mip::data_shared::DATA_SYS_TIME_DELTA};


bool is_shared_field_descriptor(uint8_t descriptor){return std::find(SHARED_FIELD_DESCRIPTORS.begin(), SHARED_FIELD_DESCRIPTORS.end(), descriptor) != SHARED_FIELD_DESCRIPTORS.end();}
bool is_gnss_descriptor(uint8_t descriptor){return std::find(GNSS_DESCRIPTOR_SETS.begin(), GNSS_DESCRIPTOR_SETS.end(), descriptor) != GNSS_DESCRIPTOR_SETS.end();}

template<typename T>
std::string get_hex_string( T i )
{
    std::stringstream stream;
    stream << "0x"
           << std::setfill ('0') << std::setw(2)
           << std::hex << i;
    return stream.str();
}


std::string get_column_header_string(uint8_t field_descriptor, int index)
{
    std::string header = get_hex_string(int(field_descriptor));
    header += " " + std::to_string(index);
    return header;
}


template<typename T>
std::string to_string(const T& input)
{
    if constexpr(std::is_enum<T>::value)
        return std::to_string(uint32_t(input));
    else
        return std::to_string(input);
}


template<typename TupleT>
void append_field_string(std::vector<std::string>& output_string_vector, const TupleT& tp)
{
    std::experimental::apply
            (
                    [&](const auto& ... restArgs) {
                        auto append_val = [](const auto& x, std::vector<std::string>& output_string_vector) {
                            output_string_vector.emplace_back(to_string(x));
                        };
                        (append_val(restArgs, output_string_vector), ...);
                    }, tp
            );
}


template<typename T>
std::vector<std::string> get_field_as_string(const mip::Field& field)
{
    T data;
    field.extract<T>(data);
    std::vector<std::string> output_string_vector;
    auto data_tuple = data.as_tuple();
    append_field_string(output_string_vector, data_tuple);
    return output_string_vector;
}


std::vector<std::string> get_field_as_string(const mip::Field& field)
{
    if (is_shared_field_descriptor(field.fieldDescriptor()))
    {
        switch (field.fieldDescriptor())
        {
            case mip::data_shared::DATA_EVENT_SOURCE: {return get_field_as_string<mip::data_shared::EventSource>(field);}
            case mip::data_shared::DATA_TICKS: {return get_field_as_string<mip::data_shared::Ticks>(field);}
            case mip::data_shared::DATA_DELTA_TICKS: {return get_field_as_string<mip::data_shared::DeltaTicks>(field);}
            case mip::data_shared::DATA_GPS_TIME: {return get_field_as_string<mip::data_shared::GpsTimestamp>(field);}
            case mip::data_shared::DATA_DELTA_TIME: {return get_field_as_string<mip::data_shared::DeltaTime>(field);}
            case mip::data_shared::DATA_REFERENCE_TIME: {return get_field_as_string<mip::data_shared::ReferenceTimestamp>(field);}
            case mip::data_shared::DATA_REF_TIME_DELTA: {return get_field_as_string<mip::data_shared::ReferenceTimeDelta>(field);}
            case mip::data_shared::DATA_EXTERNAL_TIME: {return get_field_as_string<mip::data_shared::ExternalTimestamp>(field);}
            case mip::data_shared::DATA_SYS_TIME_DELTA: {return get_field_as_string<mip::data_shared::ExternalTimeDelta>(field);}
            default: {return {};}
        }
    }

    if (is_gnss_descriptor(field.descriptorSet()))
    {
        switch (field.fieldDescriptor())
        {
            case mip::data_gnss::DATA_POSITION_LLH: {return get_field_as_string<mip::data_gnss::PosLlh>(field);}
            case mip::data_gnss::DATA_POSITION_ECEF: {return get_field_as_string<mip::data_gnss::PosEcef>(field);}
            case mip::data_gnss::DATA_VELOCITY_NED: {return get_field_as_string<mip::data_gnss::VelNed>(field);}
            case mip::data_gnss::DATA_VELOCITY_ECEF: {return get_field_as_string<mip::data_gnss::VelEcef>(field);}
            case mip::data_gnss::DATA_GPS_TIME: {return get_field_as_string<mip::data_gnss::GpsTime>(field);}
            case mip::data_gnss::DATA_UTC_TIME: {return get_field_as_string<mip::data_gnss::UtcTime>(field);}
            case mip::data_gnss::DATA_FIX_INFO: {return get_field_as_string<mip::data_gnss::FixInfo>(field);}
            default: {return {};}
        }
    }

    switch (field.descriptorSet())
    {
        case mip::data_sensor::DESCRIPTOR_SET:
        {
            switch (field.fieldDescriptor())
            {
                case mip::data_sensor::DATA_ACCEL_SCALED: {return get_field_as_string<mip::data_sensor::ScaledAccel>(field);}
                case mip::data_sensor::DATA_GYRO_SCALED: {return get_field_as_string<mip::data_sensor::ScaledGyro>(field);}
                case mip::data_sensor::DATA_MAG_SCALED: {return get_field_as_string<mip::data_sensor::ScaledMag>(field);}
                case mip::data_sensor::DATA_DELTA_THETA: {return get_field_as_string<mip::data_sensor::DeltaTheta>(field);}
                case mip::data_sensor::DATA_DELTA_VELOCITY: {return get_field_as_string<mip::data_sensor::DeltaVelocity>(field);}
                case mip::data_sensor::DATA_COMP_ORIENTATION_MATRIX: {return get_field_as_string<mip::data_sensor::CompOrientationMatrix>(field);}
                case mip::data_sensor::DATA_COMP_QUATERNION: {return get_field_as_string<mip::data_sensor::CompQuaternion>(field);}
                case mip::data_sensor::DATA_COMP_EULER_ANGLES: {return get_field_as_string<mip::data_sensor::CompEulerAngles>(field);}
                case mip::data_sensor::DATA_TIME_STAMP_GPS: {return get_field_as_string<mip::data_sensor::GpsTimestamp>(field);}
                case mip::data_sensor::DATA_PRESSURE_SCALED: {return get_field_as_string<mip::data_sensor::ScaledPressure>(field);}
                case mip::data_sensor::DATA_ODOMETER: {return get_field_as_string<mip::data_sensor::OdometerData>(field);}
                default: {return {};}
            }
        }

        case mip::data_filter::DESCRIPTOR_SET:
        {
            switch (field.fieldDescriptor())
            {
                case mip::data_filter::DATA_POS_LLH: {return get_field_as_string<mip::data_filter::PositionLlh>(field);}
                case mip::data_filter::DATA_VEL_NED: {return get_field_as_string<mip::data_filter::VelocityNed>(field);}
                case mip::data_filter::DATA_ATT_QUATERNION: {return get_field_as_string<mip::data_filter::AttitudeQuaternion>(field);}
                case mip::data_filter::DATA_ATT_MATRIX: {return get_field_as_string<mip::data_filter::AttitudeDcm>(field);}
                case mip::data_filter::DATA_ATT_EULER_ANGLES: {return get_field_as_string<mip::data_filter::EulerAngles>(field);}
                case mip::data_filter::DATA_GYRO_BIAS: {return get_field_as_string<mip::data_filter::GyroBias>(field);}
                case mip::data_filter::DATA_ACCEL_BIAS: {return get_field_as_string<mip::data_filter::AccelBias>(field);}
                case mip::data_filter::DATA_POS_UNCERTAINTY: {return get_field_as_string<mip::data_filter::PositionLlhUncertainty>(field);}
                case mip::data_filter::DATA_VEL_UNCERTAINTY: {return get_field_as_string<mip::data_filter::VelocityNedUncertainty>(field);}
                case mip::data_filter::DATA_ATT_UNCERTAINTY_EULER: {return get_field_as_string<mip::data_filter::EulerAnglesUncertainty>(field);}
                case mip::data_filter::DATA_GYRO_BIAS_UNCERTAINTY: {return get_field_as_string<mip::data_filter::GyroBiasUncertainty>(field);}
                case mip::data_filter::DATA_ACCEL_BIAS_UNCERTAINTY: {return get_field_as_string<mip::data_filter::AccelBiasUncertainty>(field);}
                case mip::data_filter::DATA_LINEAR_ACCELERATION: {return get_field_as_string<mip::data_filter::LinearAccel>(field);}
                case mip::data_filter::DATA_COMPENSATED_ANGULAR_RATE: {return get_field_as_string<mip::data_filter::CompAngularRate>(field);}
                case mip::data_filter::DATA_FILTER_STATUS: {return get_field_as_string<mip::data_filter::Status>(field);}
                case mip::data_filter::DATA_FILTER_TIMESTAMP: {return get_field_as_string<mip::data_filter::Timestamp>(field);}
                case mip::data_filter::DATA_ATT_UNCERTAINTY_QUATERNION: {return get_field_as_string<mip::data_filter::QuaternionAttitudeUncertainty>(field);}
                case mip::data_filter::DATA_GRAVITY_VECTOR: {return get_field_as_string<mip::data_filter::GravityVector>(field);}
                case mip::data_filter::DATA_COMPENSATED_ACCELERATION: {return get_field_as_string<mip::data_filter::CompAccel>(field);}
                case mip::data_filter::DATA_CLOCK_CORRECTION: {return get_field_as_string<mip::data_filter::ClockCorrection>(field);}
                case mip::data_filter::DATA_CLOCK_CORRECTION_UNCERTAINTY: {return get_field_as_string<mip::data_filter::ClockCorrectionUncertainty>(field);}
                case mip::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION: {return get_field_as_string<mip::data_filter::MultiAntennaOffsetCorrection>(field);}
                case mip::data_filter::DATA_MULTI_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY: {return get_field_as_string<mip::data_filter::MultiAntennaOffsetCorrectionUncertainty>(field);}
                case mip::data_filter::DATA_ECEF_POS_UNCERTAINTY: {return get_field_as_string<mip::data_filter::EcefPosUncertainty>(field);}
                case mip::data_filter::DATA_ECEF_VEL_UNCERTAINTY: {return get_field_as_string<mip::data_filter::EcefVelUncertainty>(field);}
                case mip::data_filter::DATA_ECEF_POS: {return get_field_as_string<mip::data_filter::EcefPos>(field);}
                case mip::data_filter::DATA_ECEF_VEL: {return get_field_as_string<mip::data_filter::EcefVel>(field);}
                case mip::data_filter::DATA_REL_POS_NED: {return get_field_as_string<mip::data_filter::RelPosNed>(field);}
                case mip::data_filter::DATA_GNSS_DUAL_ANTENNA_STATUS: {return get_field_as_string<mip::data_filter::GnssDualAntennaStatus>(field);}
//                case mip::data_filter::DATA_GNSS_POS_AID_STATUS: {return get_field_as_string<mip::data_filter::GnssPosAidStatus>(field);}
                case mip::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR: {return get_field_as_string<mip::data_filter::OdometerScaleFactorError>(field);}
                case mip::data_filter::DATA_ODOMETER_SCALE_FACTOR_ERROR_UNCERTAINTY: {return get_field_as_string<mip::data_filter::OdometerScaleFactorErrorUncertainty>(field);}
                default: {return {};}
            }
        }

        default:
            return {};
    }

    return {};
}

size_t get_field_length(const mip::Field& field)
{
    return get_field_as_string(field).size();
}
