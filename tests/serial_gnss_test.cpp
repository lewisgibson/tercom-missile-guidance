// Exercises the real NMEA serial GNSS backend without hardware by driving it
// through a pseudo-terminal (POSIX). On non-POSIX platforms the backend is a stub
// that throws, which is checked instead.

#if defined(__unix__) || defined(__APPLE__)
#define _XOPEN_SOURCE 600 // posix_openpt, grantpt, unlockpt, ptsname
#endif

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "tercom/sensors/measurements.h"
#include "tercom/sensors/serial_gnss.h"

#if defined(__unix__) || defined(__APPLE__)

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

namespace {

    using TERCOM::Sensors::GnssFix;
    using TERCOM::Sensors::SerialNmeaGnss;

    TEST(SerialNmeaGnss, RejectsUnsupportedBaud) {
        EXPECT_THROW(SerialNmeaGnss("/dev/null", 1234), std::invalid_argument);
    }

    TEST(SerialNmeaGnss, ThrowsWhenDeviceCannotBeOpened) {
        EXPECT_THROW(SerialNmeaGnss("/no/such/tercom/device", 9600), std::runtime_error);
    }

    TEST(SerialNmeaGnss, ThrowsWhenDeviceIsNotATerminal) {
        // /dev/null opens but is not a terminal, so the attribute query fails.
        EXPECT_THROW(SerialNmeaGnss("/dev/null", 9600), std::runtime_error);
    }

    TEST(SerialNmeaGnss, ParsesSentencesFromAPseudoTerminal) {
        const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
        ASSERT_GE(master, 0);
        ASSERT_EQ(::grantpt(master), 0);
        ASSERT_EQ(::unlockpt(master), 0);
        const char* slave_name = ::ptsname(master);
        ASSERT_NE(slave_name, nullptr);

        SerialNmeaGnss gnss(slave_name, 9600);

        const std::string sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        const ssize_t written = ::write(master, sentence.data(), sentence.size());
        ASSERT_EQ(static_cast<std::size_t>(written), sentence.size());

        GnssFix fix;
        for (int attempt = 0; attempt < 100 && !fix.valid; ++attempt) {
            fix = gnss.Read();
        }
        ::close(master);

        ASSERT_TRUE(fix.valid);
        EXPECT_NEAR(fix.position.latitude_deg, 48.1173, 1e-3);
        EXPECT_NEAR(fix.position.longitude_deg, 11.5167, 1e-3);
        EXPECT_EQ(fix.satellites, 8);
    }

    TEST(SerialNmeaGnss, RejectsGarbageAndKeepsLastFix) {
        const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
        ASSERT_GE(master, 0);
        ASSERT_EQ(::grantpt(master), 0);
        ASSERT_EQ(::unlockpt(master), 0);
        SerialNmeaGnss gnss(::ptsname(master), 9600);

        // First establish a valid fix.
        const std::string good = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        ASSERT_EQ(static_cast<std::size_t>(::write(master, good.data(), good.size())), good.size());
        GnssFix fix;
        for (int attempt = 0; attempt < 100 && !fix.valid; ++attempt) {
            fix = gnss.Read();
        }
        ASSERT_TRUE(fix.valid);
        const GnssFix kept = fix;

        // Then feed garbage and drain it. The garbage is parsed and rejected, so the
        // last valid fix must be retained unchanged (not replaced by an invalid one).
        const std::string junk = "not a sentence\r\n$GPRMC,ignored*00\r\n";
        ASSERT_GT(::write(master, junk.data(), junk.size()), 0);
        for (int attempt = 0; attempt < 100; ++attempt) {
            fix = gnss.Read();
        }
        ::close(master);

        EXPECT_TRUE(fix.valid);
        EXPECT_DOUBLE_EQ(fix.position.latitude_deg, kept.position.latitude_deg);
        EXPECT_DOUBLE_EQ(fix.position.longitude_deg, kept.position.longitude_deg);
        EXPECT_EQ(fix.satellites, kept.satellites);
    }

    TEST(SerialNmeaGnss, AcceptsEverySupportedBaudRate) {
        for (const unsigned baud : {4800U, 9600U, 19200U, 38400U, 57600U, 115200U}) {
            const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
            ASSERT_GE(master, 0);
            ASSERT_EQ(::grantpt(master), 0);
            ASSERT_EQ(::unlockpt(master), 0);
            EXPECT_NO_THROW({ SerialNmeaGnss gnss(::ptsname(master), baud); });
            ::close(master);
        }
    }

    TEST(SerialNmeaGnss, ClearsOversizedUnterminatedBufferAndRecovers) {
        const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
        ASSERT_GE(master, 0);
        ASSERT_EQ(::grantpt(master), 0);
        ASSERT_EQ(::unlockpt(master), 0);
        SerialNmeaGnss gnss(::ptsname(master), 9600);

        // Feed more than the 4096-byte guard with no newline, draining between writes
        // so the cumulative unterminated buffer trips the overflow clear instead of
        // growing without bound or wedging the parser.
        const std::string garbage(512, 'x');
        for (int i = 0; i < 12; ++i) { // 12 * 512 = 6144 bytes > 4096, no '\n'
            ASSERT_GT(::write(master, garbage.data(), garbage.size()), 0);
            gnss.Read();
        }

        // A valid sentence must still parse, proving the parser recovered. The
        // leading newline terminates any residual unterminated garbage so the GGA
        // starts on a clean line.
        const std::string good = "\r\n$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        ASSERT_EQ(static_cast<std::size_t>(::write(master, good.data(), good.size())), good.size());
        GnssFix fix;
        for (int attempt = 0; attempt < 100 && !fix.valid; ++attempt) {
            fix = gnss.Read();
        }
        ::close(master);

        ASSERT_TRUE(fix.valid);
        EXPECT_EQ(fix.satellites, 8);
    }

} // namespace

#else

TEST(SerialNmeaGnss, UnsupportedOnNonPosix) {
    EXPECT_THROW(TERCOM::Sensors::SerialNmeaGnss("COM1", 9600), std::runtime_error);
}

#endif
