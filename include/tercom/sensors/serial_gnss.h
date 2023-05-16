#pragma once

// GNSS receiver backed by a real serial device speaking NMEA-0183 (for example a
// u-blox module on /dev/ttyUSB0). Non-blocking: each Read() drains whatever bytes
// are available, parses any complete GGA sentences, and returns the latest fix.
// POSIX only; construction throws if the device cannot be opened or configured.

#include <string>

#include "tercom/sensors/gnss_receiver.h"
#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Sensors {

    class SerialNmeaGnss : public IGnssReceiver {
      public:
        explicit SerialNmeaGnss(const std::string& device_path, unsigned baud_rate = 9600);
        SerialNmeaGnss(const SerialNmeaGnss&) = delete;
        SerialNmeaGnss& operator=(const SerialNmeaGnss&) = delete;
        SerialNmeaGnss(SerialNmeaGnss&&) = delete;
        SerialNmeaGnss& operator=(SerialNmeaGnss&&) = delete;
        ~SerialNmeaGnss() override;

        GnssFix Read() override;

      private:
        int file_descriptor_ = -1;
        std::string buffer_;
        GnssFix latest_{};
    };

}} // namespace TERCOM::Sensors
