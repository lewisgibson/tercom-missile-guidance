#include <gtest/gtest.h>

#include "tercom/sensors/measurements.h"
#include "tercom/sensors/nmea.h"

#include <optional>
#include <string>

namespace {

    // The canonical GGA sentence used throughout these tests.
    constexpr const char* kCanonical = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

} // namespace

TEST(NmeaChecksum, CanonicalSentenceIsValid) {
    EXPECT_TRUE(TERCOM::Sensors::NmeaChecksumValid(kCanonical));
}

TEST(NmeaChecksum, CorruptedChecksumIsInvalid) {
    const std::string corrupted = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48";
    EXPECT_FALSE(TERCOM::Sensors::NmeaChecksumValid(corrupted));
}

TEST(NmeaChecksum, MissingStarIsTreatedAsValid) {
    const std::string no_checksum = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    EXPECT_TRUE(TERCOM::Sensors::NmeaChecksumValid(no_checksum));
}

TEST(NmeaChecksum, EmptySentenceIsInvalid) {
    EXPECT_FALSE(TERCOM::Sensors::NmeaChecksumValid(""));
}

TEST(NmeaChecksum, SentenceNotStartingWithDollarIsInvalid) {
    const std::string no_dollar = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    EXPECT_FALSE(TERCOM::Sensors::NmeaChecksumValid(no_dollar));
}

TEST(NmeaChecksum, TruncatedChecksumFieldIsInvalid) {
    // A '*' with fewer than two trailing hex characters cannot be validated.
    const std::string truncated = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*4";
    EXPECT_FALSE(TERCOM::Sensors::NmeaChecksumValid(truncated));
}

TEST(NmeaChecksum, NonHexChecksumIsInvalid) {
    const std::string bad_hex = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*ZZ";
    EXPECT_FALSE(TERCOM::Sensors::NmeaChecksumValid(bad_hex));
}

TEST(NmeaChecksum, TrailingCarriageReturnAndLineFeedTolerated) {
    const std::string with_eol = std::string(kCanonical) + "\r\n";
    EXPECT_TRUE(TERCOM::Sensors::NmeaChecksumValid(with_eol));
}

TEST(ParseGgaSentence, CanonicalSentenceProducesValidFix) {
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(kCanonical);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);

    // 48 degrees + 7.038/60 minutes.
    EXPECT_NEAR(fix->position.latitude_deg, 48.1173, 1e-3);
    // 11 degrees + 31.000/60 minutes.
    EXPECT_NEAR(fix->position.longitude_deg, 11.5167, 1e-3);
    EXPECT_NEAR(fix->position.altitude_m, 545.4, 1e-3);
    EXPECT_EQ(fix->satellites, 8);
}

TEST(ParseGgaSentence, SouthernAndWesternHemispheresAreNegative) {
    // Same magnitudes but S latitude and W longitude. Recomputed checksum.
    const std::string southern_western = "$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(southern_western);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);
    EXPECT_NEAR(fix->position.latitude_deg, -48.1173, 1e-3);
    EXPECT_NEAR(fix->position.longitude_deg, -11.5167, 1e-3);
}

TEST(ParseGgaSentence, CorruptedChecksumReturnsNullopt) {
    const std::string corrupted = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(corrupted);
    EXPECT_FALSE(fix.has_value());
}

TEST(ParseGgaSentence, ZeroFixQualityReturnsNullopt) {
    // Fix quality field (index 6) set to 0 means no fix. Checksum recomputed.
    const std::string no_fix = "$GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*46";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(no_fix);
    EXPECT_FALSE(fix.has_value());
}

TEST(ParseGgaSentence, NonGgaSentenceReturnsNullopt) {
    // A valid RMC sentence is not a GGA sentence.
    const std::string rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(rmc);
    EXPECT_FALSE(fix.has_value());
}

TEST(ParseGgaSentence, TrailingCarriageReturnAndLineFeedTolerated) {
    const std::string with_eol = std::string(kCanonical) + "\r\n";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(with_eol);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);
    EXPECT_NEAR(fix->position.latitude_deg, 48.1173, 1e-3);
    EXPECT_EQ(fix->satellites, 8);
}

TEST(ParseGgaSentence, EmptyLatitudeFieldReturnsNullopt) {
    // Position value missing; checksum recomputed for the shortened body.
    const std::string empty_lat = "$GPGGA,123519,,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(empty_lat);
    EXPECT_FALSE(fix.has_value());
}

TEST(ParseGgaSentence, TooFewFieldsReturnsNullopt) {
    // A GGA header with far too few fields cannot yield a position.
    const std::string short_sentence = "$GPGGA,123519,4807.038,N";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(short_sentence);
    EXPECT_FALSE(fix.has_value());
}

TEST(ParseGgaSentence, AlternativeTalkerIdIsAccepted) {
    // GLONASS talker id "GL" still ends in GGA, so it must parse.
    const std::string glonass = "$GLGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*5B";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(glonass);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);
    EXPECT_NEAR(fix->position.latitude_deg, 48.1173, 1e-3);
}

TEST(ParseGgaSentence, LeadingAndTrailingWhitespaceIsTrimmed) {
    const std::string padded = std::string("  \t") + kCanonical + " \r\n";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(padded);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);
    EXPECT_EQ(fix->satellites, 8);
}

TEST(ParseGgaSentence, MalformedNumericFieldsFallBackToDefaults) {
    // No checksum (so it is accepted), valid fix quality and position, but the
    // satellite count and altitude are non-numeric and must default to zero.
    const std::string sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,zz,0.9,abc,M,46.9,M,,";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(sentence);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);
    EXPECT_EQ(fix->satellites, 0);
    EXPECT_DOUBLE_EQ(fix->position.altitude_m, 0.0);
    EXPECT_NEAR(fix->position.latitude_deg, 48.1173, 1e-3);
}

TEST(ParseGgaSentence, NonNumericFixQualityIsRejected) {
    const std::string sentence = "$GPGGA,123519,4807.038,N,01131.000,E,x,08,0.9,545.4,M,46.9,M,,";
    EXPECT_FALSE(TERCOM::Sensors::ParseGgaSentence(sentence).has_value());
}

TEST(ParseGgaSentence, EmptyFixQualityFieldReturnsNullopt) {
    // Empty fix-quality field (index 6); no checksum so the sentence is accepted.
    const std::string sentence = "$GPGGA,123519,4807.038,N,01131.000,E,,08,0.9,545.4,M,46.9,M,,";
    EXPECT_FALSE(TERCOM::Sensors::ParseGgaSentence(sentence).has_value());
}

TEST(ParseGgaSentence, EmptySatelliteFieldDefaultsToZero) {
    // Empty satellite-count field (index 7) parses but defaults the count to zero.
    const std::string sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,,0.9,545.4,M,46.9,M,,";
    const std::optional<TERCOM::Sensors::GnssFix> fix = TERCOM::Sensors::ParseGgaSentence(sentence);
    ASSERT_TRUE(fix.has_value());
    EXPECT_TRUE(fix->valid);
    EXPECT_EQ(fix->satellites, 0);
}
