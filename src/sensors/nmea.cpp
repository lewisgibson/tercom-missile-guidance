#include "tercom/sensors/nmea.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <vector>

namespace TERCOM { namespace Sensors {

    namespace {

        std::string_view Trim(std::string_view text) {
            while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) {
                text.remove_prefix(1);
            }
            while (!text.empty() && (text.back() == '\r' || text.back() == '\n' || text.back() == ' ' || text.back() == '\t')) {
                text.remove_suffix(1);
            }
            return text;
        }

        std::optional<double> ParseDouble(std::string_view field) {
            if (field.empty()) {
                return std::nullopt;
            }
            double value = 0.0;
            const auto* end = field.data() + field.size();
            const auto result = std::from_chars(field.data(), end, value);
            if (result.ec != std::errc{} || result.ptr != end) {
                return std::nullopt;
            }
            return value;
        }

        std::optional<int> ParseInt(std::string_view field) {
            if (field.empty()) {
                return std::nullopt;
            }
            int value = 0;
            const auto* end = field.data() + field.size();
            const auto result = std::from_chars(field.data(), end, value);
            if (result.ec != std::errc{} || result.ptr != end) {
                return std::nullopt;
            }
            return value;
        }

        // Convert NMEA ddmm.mmmm / dddmm.mmmm to signed decimal degrees. NMEA packs
        // a coordinate as whole degrees in the leading two digits (latitude) or three
        // digits (longitude) immediately followed by minutes, for example 5123.456
        // means 51 degrees and 23.456 minutes. So the integer part divided by 100 is
        // the degrees, the remainder is the minutes, and minutes over 60 converts to
        // a degree fraction. The separate hemisphere letter (N/S, E/W) gives the sign,
        // since the numeric field itself is always positive.
        std::optional<double> ParseDegrees(std::string_view value_field, std::string_view hemisphere) {
            const std::optional<double> raw = ParseDouble(value_field);
            if (!raw.has_value()) {
                return std::nullopt;
            }
            const double degrees = std::floor(*raw / 100.0);
            const double minutes = *raw - (degrees * 100.0);
            double decimal = degrees + (minutes / 60.0);
            if (hemisphere == "S" || hemisphere == "W") {
                decimal = -decimal;
            }
            return decimal;
        }

    } // namespace

    bool NmeaChecksumValid(std::string_view sentence) {
        const std::string_view trimmed = Trim(sentence);
        if (trimmed.empty() || trimmed.front() != '$') {
            return false;
        }

        const auto star = trimmed.find('*');
        if (star == std::string_view::npos) {
            return true; // checksum omitted
        }
        if (star + 2 >= trimmed.size()) {
            return false;
        }

        // The checksum is the XOR of every byte strictly between the leading '$'
        // (index 0) and the '*', so start at index 1 and stop before the star.
        std::uint8_t computed = 0;
        for (std::size_t i = 1; i < star; ++i) {
            computed ^= static_cast<std::uint8_t>(trimmed[i]);
        }

        // The two characters after '*' are the expected checksum in hexadecimal;
        // base 16 is passed to from_chars to parse them.
        const std::string_view checksum_field = trimmed.substr(star + 1, 2);
        unsigned expected = 0;
        const auto* end = checksum_field.data() + checksum_field.size();
        const auto result = std::from_chars(checksum_field.data(), end, expected, 16);
        if (result.ec != std::errc{} || result.ptr != end) {
            return false;
        }
        return computed == expected;
    }

    std::optional<GnssFix> ParseGgaSentence(std::string_view sentence) {
        const std::string_view trimmed = Trim(sentence);
        if (!NmeaChecksumValid(trimmed)) {
            return std::nullopt;
        }

        // Drop any checksum suffix before splitting into fields.
        std::string_view body = trimmed;
        if (const auto star = body.find('*'); star != std::string_view::npos) {
            body = body.substr(0, star);
        }

        std::vector<std::string_view> fields;
        std::size_t start = 0;
        while (start <= body.size()) {
            const auto comma = body.find(',', start);
            if (comma == std::string_view::npos) {
                fields.push_back(body.substr(start));
                break;
            }
            fields.push_back(body.substr(start, comma - start));
            start = comma + 1;
        }

        // GGA fields by index: 0 talker id "$xxGGA", 1 UTC time, 2 latitude
        // ddmm.mmmm, 3 N/S, 4 longitude dddmm.mmmm, 5 E/W, 6 fix quality, 7 number of
        // satellites, 8 horizontal dilution, 9 altitude (metres), then geoid
        // separation and DGPS fields not used here. Field 0 is "$xxGGA"; GGA is the
        // last three characters of the talker id.
        if (fields.empty() || fields[0].size() < 3 || fields[0].substr(fields[0].size() - 3) != "GGA") {
            return std::nullopt;
        }
        if (fields.size() < 10) {
            return std::nullopt;
        }

        // Fix quality 0 means "no fix"; anything nonzero is a usable solution.
        const std::optional<int> quality = ParseInt(fields[6]);
        if (!quality.has_value() || *quality == 0) {
            return std::nullopt; // no valid fix
        }

        const std::optional<double> latitude = ParseDegrees(fields[2], fields[3]);
        const std::optional<double> longitude = ParseDegrees(fields[4], fields[5]);
        if (!latitude.has_value() || !longitude.has_value()) {
            return std::nullopt;
        }

        GnssFix fix;
        fix.position.latitude_deg = *latitude;
        fix.position.longitude_deg = *longitude;
        fix.position.altitude_m = ParseDouble(fields[9]).value_or(0.0);
        fix.satellites = ParseInt(fields[7]).value_or(0);
        fix.valid = true;
        return fix;
    }

}} // namespace TERCOM::Sensors
