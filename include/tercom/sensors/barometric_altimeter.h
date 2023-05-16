#pragma once

#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Sensors {

    // Barometric altimeter providing absolute altitude above the local ENU reference
    // datum (the tangent plane at the geodetic origin). Combined with radar ground
    // clearance it yields the terrain elevation beneath the vehicle, which is what
    // TERCOM correlates.
    class IBarometricAltimeter {
      public:
        virtual ~IBarometricAltimeter() = default;
        virtual BarometricSample Read() = 0;
    };

}} // namespace TERCOM::Sensors
