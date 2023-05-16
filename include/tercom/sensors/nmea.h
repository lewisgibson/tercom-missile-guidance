#pragma once

// NMEA-0183 parsing for satellite receivers. NMEA-0183 is the de facto serial
// text protocol GNSS modules speak: one ASCII "sentence" per line, beginning with
// '$', fields separated by commas, optionally ending with '*' and a two-hex-digit
// checksum. This parser handles the GGA sentence (essential fix data: time,
// position, fix quality, satellite count, and altitude). The parser is a pure
// function so it can be unit-tested without any hardware; the serial backend
// (serial_gnss.h) feeds it lines read from a real device.

#include <optional>
#include <string_view>

#include "tercom/sensors/measurements.h"

namespace TERCOM { namespace Sensors {

    // Validate the NMEA checksum: the XOR of every character between '$' and '*'
    // must equal the two hex digits following '*'. Sentences without a '*' are
    // treated as valid (some emitters omit it).
    bool NmeaChecksumValid(std::string_view sentence);

    // Parse a GGA sentence ("$..GGA,...") into a geodetic fix. Returns nullopt for
    // other sentence types, a failed checksum, or a malformed/empty position.
    std::optional<GnssFix> ParseGgaSentence(std::string_view sentence);

}} // namespace TERCOM::Sensors
