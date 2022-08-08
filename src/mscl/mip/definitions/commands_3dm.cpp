
#include "commands_3dm.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;
namespace commands_3dm {

namespace C {
struct mip_interface;
} // namespace C

using ::mscl::insert;
using ::mscl::extract;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const NMEAMessageFormat& self)
{
    insert(serializer, self.message_id);
    insert(serializer, self.talker_id);
    insert(serializer, self.source_id);
    insert(serializer, self.decimation);
}

void extract(MipSerializer& serializer, NMEAMessageFormat& self)
{
    extract(serializer, self.message_id);
    extract(serializer, self.talker_id);
    extract(serializer, self.source_id);
    extract(serializer, self.decimation);
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const PollImuMessage& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollImuMessage& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const PollGnssMessage& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollGnssMessage& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const PollFilterMessage& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollFilterMessage& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const ImuMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, ImuMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const GpsMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, GpsMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const FilterMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, FilterMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, ImuGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, GpsGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, FilterGetBaseRate& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const PollData& self)
{
    insert(serializer, self.desc_set);
    insert(serializer, self.suppress_ack);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, PollData& self)
{
    extract(serializer, self.desc_set);
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const GetBaseRate& self)
{
    insert(serializer, self.desc_set);
}

void extract(MipSerializer& serializer, GetBaseRate& self)
{
    extract(serializer, self.desc_set);
}

void insert(MipSerializer& serializer, const MessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.desc_set);
    insert(serializer, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        insert(serializer, self.descriptors[i]);
}

void extract(MipSerializer& serializer, MessageFormat& self)
{
    extract(serializer, self.function);
    extract(serializer, self.desc_set);
    mscl::C::extract_count(&serializer, &self.num_descriptors, self.num_descriptors);
    for(unsigned int i=0; i < self.num_descriptors; i++)
        extract(serializer, self.descriptors[i]);
}

void insert(MipSerializer& serializer, const NmeaPollData& self)
{
    insert(serializer, self.suppress_ack);
    insert(serializer, self.count);
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
}

void extract(MipSerializer& serializer, NmeaPollData& self)
{
    extract(serializer, self.suppress_ack);
    mscl::C::extract_count(&serializer, &self.count, self.count);
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
}

void insert(MipSerializer& serializer, const NmeaMessageFormat& self)
{
    insert(serializer, self.function);
    insert(serializer, self.count);
    for(unsigned int i=0; i < self.count; i++)
        insert(serializer, self.format_entries[i]);
}

void extract(MipSerializer& serializer, NmeaMessageFormat& self)
{
    extract(serializer, self.function);
    mscl::C::extract_count(&serializer, &self.count, self.count);
    for(unsigned int i=0; i < self.count; i++)
        extract(serializer, self.format_entries[i]);
}

void insert(MipSerializer& serializer, const DeviceSettings& self)
{
    insert(serializer, self.function);
}

void extract(MipSerializer& serializer, DeviceSettings& self)
{
    extract(serializer, self.function);
}

void insert(MipSerializer& serializer, const UartBaudrate& self)
{
    insert(serializer, self.function);
    insert(serializer, self.baud);
}

void extract(MipSerializer& serializer, UartBaudrate& self)
{
    extract(serializer, self.function);
    extract(serializer, self.baud);
}

void insert(MipSerializer& serializer, const FactoryStreaming& self)
{
    insert(serializer, self.action);
    insert(serializer, self.reserved);
}

void extract(MipSerializer& serializer, FactoryStreaming& self)
{
    extract(serializer, self.action);
    extract(serializer, self.reserved);
}

void insert(MipSerializer& serializer, const DatastreamControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.desc_set);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, DatastreamControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.desc_set);
    extract(serializer, self.enable);
}

void insert(MipSerializer& serializer, const GnssSbasSettings& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable_sbas);
    insert(serializer, self.sbas_options);
    insert(serializer, self.num_included_prns);
    for(unsigned int i=0; i < self.num_included_prns; i++)
        insert(serializer, self.included_prns[i]);
}

void extract(MipSerializer& serializer, GnssSbasSettings& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable_sbas);
    extract(serializer, self.sbas_options);
    mscl::C::extract_count(&serializer, &self.num_included_prns, self.num_included_prns);
    for(unsigned int i=0; i < self.num_included_prns; i++)
        extract(serializer, self.included_prns[i]);
}

void insert(MipSerializer& serializer, const GnssTimeAssistance& self)
{
    insert(serializer, self.function);
    insert(serializer, self.tow);
    insert(serializer, self.week_number);
    insert(serializer, self.accuracy);
}

void extract(MipSerializer& serializer, GnssTimeAssistance& self)
{
    extract(serializer, self.function);
    extract(serializer, self.tow);
    extract(serializer, self.week_number);
    extract(serializer, self.accuracy);
}

void insert(MipSerializer& serializer, const AdvLowpassFilter& self)
{
    insert(serializer, self.function);
    insert(serializer, self.target_descriptor);
    insert(serializer, self.enable);
    insert(serializer, self.manual);
    insert(serializer, self.frequency);
    insert(serializer, self.reserved);
}

void extract(MipSerializer& serializer, AdvLowpassFilter& self)
{
    extract(serializer, self.function);
    extract(serializer, self.target_descriptor);
    extract(serializer, self.enable);
    extract(serializer, self.manual);
    extract(serializer, self.frequency);
    extract(serializer, self.reserved);
}

void insert(MipSerializer& serializer, const PpsSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
}

void extract(MipSerializer& serializer, PpsSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
}

void insert(MipSerializer& serializer, const GpioConfig& self)
{
    insert(serializer, self.function);
    insert(serializer, self.pin);
    insert(serializer, self.feature);
    insert(serializer, self.behavior);
    insert(serializer, self.pin_mode);
}

void extract(MipSerializer& serializer, GpioConfig& self)
{
    extract(serializer, self.function);
    extract(serializer, self.pin);
    extract(serializer, self.feature);
    extract(serializer, self.behavior);
    extract(serializer, self.pin_mode);
}

void insert(MipSerializer& serializer, const GpioState& self)
{
    insert(serializer, self.function);
    insert(serializer, self.pin);
    insert(serializer, self.state);
}

void extract(MipSerializer& serializer, GpioState& self)
{
    extract(serializer, self.function);
    extract(serializer, self.pin);
    extract(serializer, self.state);
}

void insert(MipSerializer& serializer, const Odometer& self)
{
    insert(serializer, self.function);
    insert(serializer, self.mode);
    insert(serializer, self.scaling);
    insert(serializer, self.uncertainty);
}

void extract(MipSerializer& serializer, Odometer& self)
{
    extract(serializer, self.function);
    extract(serializer, self.mode);
    extract(serializer, self.scaling);
    extract(serializer, self.uncertainty);
}

void insert(MipSerializer& serializer, const GetEventSupport& self)
{
    insert(serializer, self.query);
}

void extract(MipSerializer& serializer, GetEventSupport& self)
{
    extract(serializer, self.query);
}

void insert(MipSerializer& serializer, const EventControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.instance);
    insert(serializer, self.mode);
}

void extract(MipSerializer& serializer, EventControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.instance);
    extract(serializer, self.mode);
}

void insert(MipSerializer& serializer, const GetEventTriggerStatus& self)
{
    insert(serializer, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
}

void extract(MipSerializer& serializer, GetEventTriggerStatus& self)
{
    mscl::C::extract_count(&serializer, &self.requested_count, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
}

void insert(MipSerializer& serializer, const GetEventActionStatus& self)
{
    insert(serializer, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        insert(serializer, self.requested_instances[i]);
}

void extract(MipSerializer& serializer, GetEventActionStatus& self)
{
    mscl::C::extract_count(&serializer, &self.requested_count, self.requested_count);
    for(unsigned int i=0; i < self.requested_count; i++)
        extract(serializer, self.requested_instances[i]);
}

void insert(MipSerializer& serializer, const EventTrigger& self)
{
    insert(serializer, self.function);
    insert(serializer, self.instance);
    insert(serializer, self.type);
    if( self.type == EventTrigger::Type::GPIO )
        insert(serializer, self.gpio);
    if( self.type == EventTrigger::Type::THRESHOLD )
        insert(serializer, self.threshold);
    if( self.type == EventTrigger::Type::COMBINATION )
        insert(serializer, self.combination);
}

void extract(MipSerializer& serializer, EventTrigger& self)
{
    extract(serializer, self.function);
    extract(serializer, self.instance);
    extract(serializer, self.type);
    if( self.type == EventTrigger::Type::GPIO )
        extract(serializer, self.gpio);
    if( self.type == EventTrigger::Type::THRESHOLD )
        extract(serializer, self.threshold);
    if( self.type == EventTrigger::Type::COMBINATION )
        extract(serializer, self.combination);
}

void insert(MipSerializer& serializer, const EventAction& self)
{
    insert(serializer, self.function);
    insert(serializer, self.instance);
    insert(serializer, self.trigger);
    insert(serializer, self.type);
    if( self.type == EventAction::Type::GPIO )
        insert(serializer, self.gpio);
    if( self.type == EventAction::Type::MESSAGE )
        insert(serializer, self.message);
}

void extract(MipSerializer& serializer, EventAction& self)
{
    extract(serializer, self.function);
    extract(serializer, self.instance);
    extract(serializer, self.trigger);
    extract(serializer, self.type);
    if( self.type == EventAction::Type::GPIO )
        extract(serializer, self.gpio);
    if( self.type == EventAction::Type::MESSAGE )
        extract(serializer, self.message);
}

void insert(MipSerializer& serializer, const AccelBias& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
}

void extract(MipSerializer& serializer, AccelBias& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
}

void insert(MipSerializer& serializer, const GyroBias& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.bias[i]);
}

void extract(MipSerializer& serializer, GyroBias& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.bias[i]);
}

void insert(MipSerializer& serializer, const CaptureGyroBias& self)
{
    insert(serializer, self.averaging_time_ms);
}

void extract(MipSerializer& serializer, CaptureGyroBias& self)
{
    extract(serializer, self.averaging_time_ms);
}

void insert(MipSerializer& serializer, const MagHardIronOffset& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
}

void extract(MipSerializer& serializer, MagHardIronOffset& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
}

void insert(MipSerializer& serializer, const MagSoftIronMatrix& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.offset[i]);
}

void extract(MipSerializer& serializer, MagSoftIronMatrix& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.offset[i]);
}

void insert(MipSerializer& serializer, const Sensor2VehicleTransformEuler& self)
{
    insert(serializer, self.function);
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
}

void extract(MipSerializer& serializer, Sensor2VehicleTransformEuler& self)
{
    extract(serializer, self.function);
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
}

void insert(MipSerializer& serializer, const Sensor2VehicleTransformQuaternion& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
}

void extract(MipSerializer& serializer, Sensor2VehicleTransformQuaternion& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
}

void insert(MipSerializer& serializer, const Sensor2VehicleTransformDcm& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
}

void extract(MipSerializer& serializer, Sensor2VehicleTransformDcm& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
}

void insert(MipSerializer& serializer, const ComplementaryFilter& self)
{
    insert(serializer, self.function);
    insert(serializer, self.pitch_roll_enable);
    insert(serializer, self.heading_enable);
    insert(serializer, self.pitch_roll_time_constant);
    insert(serializer, self.heading_time_constant);
}

void extract(MipSerializer& serializer, ComplementaryFilter& self)
{
    extract(serializer, self.function);
    extract(serializer, self.pitch_roll_enable);
    extract(serializer, self.heading_enable);
    extract(serializer, self.pitch_roll_time_constant);
    extract(serializer, self.heading_time_constant);
}

void insert(MipSerializer& serializer, const SensorRange& self)
{
    insert(serializer, self.function);
    insert(serializer, self.sensor);
    insert(serializer, self.setting);
}

void extract(MipSerializer& serializer, SensorRange& self)
{
    extract(serializer, self.function);
    extract(serializer, self.sensor);
    extract(serializer, self.setting);
}

void insert(MipSerializer& serializer, const CalibratedSensorRanges& self)
{
    insert(serializer, self.sensor);
}

void extract(MipSerializer& serializer, CalibratedSensorRanges& self)
{
    extract(serializer, self.sensor);
}


} // namespace commands_3dm
} // namespace mscl

