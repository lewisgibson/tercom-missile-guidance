#pragma once

namespace TERCOM { namespace Trajectory {

    // Commanded control corrections produced by the guidance law to fly the
    // planned route. Angles are in radians; throttle is normalised to [0, 1]
    // where 0 is idle and 1 is full power. These are setpoints for an autopilot
    // or flight model to track, not direct actuator deflections.
    struct Correction {
        double roll_command_rad = 0.0;  // commanded bank angle; banking turns the vehicle (positive rolls right)
        double pitch_command_rad = 0.0; // commanded pitch; positive noses up to climb, negative to descend
        double yaw_command_rad = 0.0;   // heading error to the desired course (left for downstream yaw control)
        double throttle = 0.0;          // normalised power demand, 0 (idle) to 1 (full)
    };

}} // namespace TERCOM::Trajectory
