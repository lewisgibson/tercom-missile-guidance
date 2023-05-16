#pragma once

#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Sensors {

    // Downward-looking radar altimeter measuring height above the terrain.
    class IRadarAltimeter {
      public:
        virtual ~IRadarAltimeter() = default;
        virtual RadarAltitudeSample Read() = 0;
    };

}} // namespace TERCOM::Sensors
