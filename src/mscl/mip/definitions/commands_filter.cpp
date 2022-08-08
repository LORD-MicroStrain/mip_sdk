
#include "commands_filter.hpp"

#include "../../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mscl {
class MipSerializer;
namespace commands_filter {

namespace C {
struct mip_interface;
} // namespace C

using ::mscl::insert;
using ::mscl::extract;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(MipSerializer& serializer, const Reset& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, Reset& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const SetInitialAttitude& self)
{
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.heading);
}

void extract(MipSerializer& serializer, SetInitialAttitude& self)
{
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.heading);
}

void insert(MipSerializer& serializer, const EstimationControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, EstimationControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
}

void insert(MipSerializer& serializer, const ExternalGnssUpdate& self)
{
    insert(serializer, self.gps_time);
    insert(serializer, self.gps_week);
    insert(serializer, self.latitude);
    insert(serializer, self.longitude);
    insert(serializer, self.height);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.vel_uncertainty[i]);
}

void extract(MipSerializer& serializer, ExternalGnssUpdate& self)
{
    extract(serializer, self.gps_time);
    extract(serializer, self.gps_week);
    extract(serializer, self.latitude);
    extract(serializer, self.longitude);
    extract(serializer, self.height);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.pos_uncertainty[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.vel_uncertainty[i]);
}

void insert(MipSerializer& serializer, const ExternalHeadingUpdate& self)
{
    insert(serializer, self.heading);
    insert(serializer, self.heading_uncertainty);
    insert(serializer, self.type);
}

void extract(MipSerializer& serializer, ExternalHeadingUpdate& self)
{
    extract(serializer, self.heading);
    extract(serializer, self.heading_uncertainty);
    extract(serializer, self.type);
}

void insert(MipSerializer& serializer, const ExternalHeadingUpdateWithTime& self)
{
    insert(serializer, self.gps_time);
    insert(serializer, self.gps_week);
    insert(serializer, self.heading);
    insert(serializer, self.heading_uncertainty);
    insert(serializer, self.type);
}

void extract(MipSerializer& serializer, ExternalHeadingUpdateWithTime& self)
{
    extract(serializer, self.gps_time);
    extract(serializer, self.gps_week);
    extract(serializer, self.heading);
    extract(serializer, self.heading_uncertainty);
    extract(serializer, self.type);
}

void insert(MipSerializer& serializer, const TareOrientation& self)
{
    insert(serializer, self.function);
    insert(serializer, self.axes);
}

void extract(MipSerializer& serializer, TareOrientation& self)
{
    extract(serializer, self.function);
    extract(serializer, self.axes);
}

void insert(MipSerializer& serializer, const SensorToVehicleRotationEuler& self)
{
    insert(serializer, self.function);
    insert(serializer, self.roll);
    insert(serializer, self.pitch);
    insert(serializer, self.yaw);
}

void extract(MipSerializer& serializer, SensorToVehicleRotationEuler& self)
{
    extract(serializer, self.function);
    extract(serializer, self.roll);
    extract(serializer, self.pitch);
    extract(serializer, self.yaw);
}

void insert(MipSerializer& serializer, const SensorToVehicleRotationDcm& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
}

void extract(MipSerializer& serializer, SensorToVehicleRotationDcm& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
}

void insert(MipSerializer& serializer, const SensorToVehicleRotationQuaternion& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.quat[i]);
}

void extract(MipSerializer& serializer, SensorToVehicleRotationQuaternion& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.quat[i]);
}

void insert(MipSerializer& serializer, const SensorToVehicleOffset& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
}

void extract(MipSerializer& serializer, SensorToVehicleOffset& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
}

void insert(MipSerializer& serializer, const AntennaOffset& self)
{
    insert(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
}

void extract(MipSerializer& serializer, AntennaOffset& self)
{
    extract(serializer, self.function);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
}

void insert(MipSerializer& serializer, const GnssSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
}

void extract(MipSerializer& serializer, GnssSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
}

void insert(MipSerializer& serializer, const HeadingSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
}

void extract(MipSerializer& serializer, HeadingSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
}

void insert(MipSerializer& serializer, const AltitudeAiding& self)
{
    insert(serializer, self.function);
    insert(serializer, self.aiding_selector);
}

void extract(MipSerializer& serializer, AltitudeAiding& self)
{
    extract(serializer, self.function);
    extract(serializer, self.aiding_selector);
}

void insert(MipSerializer& serializer, const AutoZupt& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    insert(serializer, self.threshold);
}

void extract(MipSerializer& serializer, AutoZupt& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    extract(serializer, self.threshold);
}

void insert(MipSerializer& serializer, const AutoAngularZupt& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    insert(serializer, self.threshold);
}

void extract(MipSerializer& serializer, AutoAngularZupt& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    extract(serializer, self.threshold);
}

void insert(MipSerializer& serializer, const CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const AidingMeasurementEnable& self)
{
    insert(serializer, self.function);
    insert(serializer, self.aiding_source);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, AidingMeasurementEnable& self)
{
    extract(serializer, self.function);
    extract(serializer, self.aiding_source);
    extract(serializer, self.enable);
}

void insert(MipSerializer& serializer, const Run& self)
{
    (void)serializer;
    (void)self;
}

void extract(MipSerializer& serializer, Run& self)
{
    (void)serializer;
    (void)self;
}

void insert(MipSerializer& serializer, const KinematicConstraint& self)
{
    insert(serializer, self.function);
    insert(serializer, self.acceleration_constraint_selection);
    insert(serializer, self.velocity_constraint_selection);
    insert(serializer, self.angular_constraint_selection);
}

void extract(MipSerializer& serializer, KinematicConstraint& self)
{
    extract(serializer, self.function);
    extract(serializer, self.acceleration_constraint_selection);
    extract(serializer, self.velocity_constraint_selection);
    extract(serializer, self.angular_constraint_selection);
}

void insert(MipSerializer& serializer, const InitializationConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.wait_for_run_command);
    insert(serializer, self.initial_cond_src);
    insert(serializer, self.auto_heading_alignment_selector);
    insert(serializer, self.initial_heading);
    insert(serializer, self.initial_pitch);
    insert(serializer, self.initial_roll);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_velocity[i]);
    insert(serializer, self.reference_frame_selector);
}

void extract(MipSerializer& serializer, InitializationConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.wait_for_run_command);
    extract(serializer, self.initial_cond_src);
    extract(serializer, self.auto_heading_alignment_selector);
    extract(serializer, self.initial_heading);
    extract(serializer, self.initial_pitch);
    extract(serializer, self.initial_roll);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_position[i]);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_velocity[i]);
    extract(serializer, self.reference_frame_selector);
}

void insert(MipSerializer& serializer, const AdaptiveFilterOptions& self)
{
    insert(serializer, self.function);
    insert(serializer, self.level);
    insert(serializer, self.time_limit);
}

void extract(MipSerializer& serializer, AdaptiveFilterOptions& self)
{
    extract(serializer, self.function);
    extract(serializer, self.level);
    extract(serializer, self.time_limit);
}

void insert(MipSerializer& serializer, const MultiAntennaOffset& self)
{
    insert(serializer, self.function);
    insert(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.antenna_offset[i]);
}

void extract(MipSerializer& serializer, MultiAntennaOffset& self)
{
    extract(serializer, self.function);
    extract(serializer, self.receiver_id);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.antenna_offset[i]);
}

void insert(MipSerializer& serializer, const RelPosConfiguration& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
    insert(serializer, self.reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reference_coordinates[i]);
}

void extract(MipSerializer& serializer, RelPosConfiguration& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
    extract(serializer, self.reference_frame_selector);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reference_coordinates[i]);
}

void insert(MipSerializer& serializer, const RefPointLeverArm& self)
{
    insert(serializer, self.function);
    insert(serializer, self.ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
}

void extract(MipSerializer& serializer, RefPointLeverArm& self)
{
    extract(serializer, self.function);
    extract(serializer, self.ref_point_sel);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
}

void insert(MipSerializer& serializer, const SpeedMeasurement& self)
{
    insert(serializer, self.source);
    insert(serializer, self.time_of_week);
    insert(serializer, self.speed);
    insert(serializer, self.speed_uncertainty);
}

void extract(MipSerializer& serializer, SpeedMeasurement& self)
{
    extract(serializer, self.source);
    extract(serializer, self.time_of_week);
    extract(serializer, self.speed);
    extract(serializer, self.speed_uncertainty);
}

void insert(MipSerializer& serializer, const SpeedLeverArm& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
}

void extract(MipSerializer& serializer, SpeedLeverArm& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
}

void insert(MipSerializer& serializer, const WheeledVehicleConstraintControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, WheeledVehicleConstraintControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
}

void insert(MipSerializer& serializer, const VerticalGyroConstraintControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
}

void extract(MipSerializer& serializer, VerticalGyroConstraintControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
}

void insert(MipSerializer& serializer, const GnssAntennaCalControl& self)
{
    insert(serializer, self.function);
    insert(serializer, self.enable);
    insert(serializer, self.max_offset);
}

void extract(MipSerializer& serializer, GnssAntennaCalControl& self)
{
    extract(serializer, self.function);
    extract(serializer, self.enable);
    extract(serializer, self.max_offset);
}

void insert(MipSerializer& serializer, const MagneticDeclinationSource& self)
{
    insert(serializer, self.function);
    insert(serializer, self.source);
    insert(serializer, self.declination);
}

void extract(MipSerializer& serializer, MagneticDeclinationSource& self)
{
    extract(serializer, self.function);
    extract(serializer, self.source);
    extract(serializer, self.declination);
}

void insert(MipSerializer& serializer, const SetInitialHeading& self)
{
    insert(serializer, self.heading);
}

void extract(MipSerializer& serializer, SetInitialHeading& self)
{
    extract(serializer, self.heading);
}


} // namespace commands_filter
} // namespace mscl

