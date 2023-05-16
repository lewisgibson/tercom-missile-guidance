#pragma once

#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Sensors {

    // Strapdown inertial measurement unit. Read() returns the most recent
    // angular-rate and specific-force sample.
    class IImu {
      public:
        virtual ~IImu() = default;
        virtual ImuSample Read() = 0;
    };

}} // namespace TERCOM::Sensors
