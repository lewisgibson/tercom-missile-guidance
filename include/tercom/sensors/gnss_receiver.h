#pragma once

#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Sensors {

    // Satellite navigation receiver. Optional in the pipeline: terrain-referenced
    // navigation is the fallback when no fix is available, so this is used only
    // for initial alignment or an independent cross-check.
    class IGnssReceiver {
      public:
        virtual ~IGnssReceiver() = default;
        virtual GnssFix Read() = 0;
    };

}} // namespace TERCOM::Sensors
