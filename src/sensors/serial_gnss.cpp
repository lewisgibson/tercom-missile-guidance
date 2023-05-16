// Enable the glibc/BSD serial extensions (cfmakeraw, CRTSCTS) under -std=c++20.
// Must precede every system header include. These are standard feature-test
// macros, hence the reserved-identifier and macro-usage suppressions.
// NOLINTBEGIN(bugprone-reserved-identifier, cert-dcl37-c, cert-dcl51-cpp, cppcoreguidelines-macro-usage)
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
// NOLINTEND(bugprone-reserved-identifier, cert-dcl37-c, cert-dcl51-cpp, cppcoreguidelines-macro-usage)

#include "tercom/sensors/serial_gnss.h"

#include <stdexcept>

#include "tercom/sensors/nmea.h"

#if defined(__unix__) || defined(__APPLE__)

#include <array>
#include <cerrno>
#include <system_error>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace TERCOM { namespace Sensors {

    namespace {
        // termios identifies line speeds by named constants (Bxxxx), not raw integer
        // baud values, so map the requested rate onto the matching constant. 9600 is
        // the long-standing NMEA-0183 default; the higher rates suit modern modules.
        speed_t BaudConstant(unsigned baud_rate) {
            switch (baud_rate) {
                case 4800:
                    return B4800;
                case 9600:
                    return B9600;
                case 19200:
                    return B19200;
                case 38400:
                    return B38400;
                case 57600:
                    return B57600;
                case 115200:
                    return B115200;
                default:
                    throw std::invalid_argument("unsupported serial baud rate");
            }
        }
    } // namespace

    SerialNmeaGnss::SerialNmeaGnss(const std::string& device_path, unsigned baud_rate) {
        const speed_t speed = BaudConstant(baud_rate);

        // Opened here, after the baud is validated, so the result can be checked;
        // hence not a member initializer.
        // Flags: O_RDONLY we only read fixes; O_NOCTTY keeps this serial port from
        // becoming the process controlling terminal (otherwise signals like Ctrl-C
        // on the line could reach us); O_NONBLOCK so open() and later reads return
        // immediately instead of waiting for data.
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        file_descriptor_ = ::open(device_path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (file_descriptor_ < 0) {
            throw std::runtime_error("could not open serial device " + device_path + ": " + std::generic_category().message(errno));
        }

        termios options{};
        if (::tcgetattr(file_descriptor_, &options) != 0) {
            const std::string message = std::generic_category().message(errno);
            ::close(file_descriptor_);
            file_descriptor_ = -1;
            throw std::runtime_error("could not read serial attributes: " + message);
        }

        // cfmakeraw puts the port in "raw" mode: bytes pass through untouched, with
        // no line editing, echo, or CR/LF translation, which is what a binary-style
        // text protocol needs. cfsetispeed/cfsetospeed set the input and output line
        // speeds to the chosen baud.
        ::cfmakeraw(&options);
        ::cfsetispeed(&options, speed);
        ::cfsetospeed(&options, speed);
        // CLOCAL: ignore modem control lines (this is not a dial-up modem). CREAD:
        // enable the receiver so bytes are actually delivered.
        options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
        // Clear CRTSCTS to disable RTS/CTS hardware flow control; GNSS modules
        // typically do not wire those handshake lines.
        options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
        // VMIN=0, VTIME=0 make read() return immediately with whatever is buffered
        // (possibly nothing) rather than blocking for a minimum byte count or timeout.
        options.c_cc[VMIN] = 0; // non-blocking
        options.c_cc[VTIME] = 0;

        // TCSANOW applies the new attributes immediately.
        if (::tcsetattr(file_descriptor_, TCSANOW, &options) != 0) {
            const std::string message = std::generic_category().message(errno);
            ::close(file_descriptor_);
            file_descriptor_ = -1;
            throw std::runtime_error("could not configure serial device: " + message);
        }
    }

    SerialNmeaGnss::~SerialNmeaGnss() {
        if (file_descriptor_ >= 0) {
            ::close(file_descriptor_);
        }
    }

    GnssFix SerialNmeaGnss::Read() {
        std::array<char, 256> chunk{};
        for (;;) {
            const ssize_t count = ::read(file_descriptor_, chunk.data(), chunk.size());
            if (count <= 0) {
                break; // no more data available right now
            }
            buffer_.append(chunk.data(), static_cast<std::size_t>(count));
        }

        // Guard against an endless line with no terminator. An NMEA sentence is at
        // most about 82 characters, so 4096 bytes without a newline means the stream
        // is garbage or out of sync; drop it rather than grow without bound.
        constexpr std::size_t kMaxBuffer = 4096;
        if (buffer_.size() > kMaxBuffer) {
            buffer_.clear();
        }

        // Consume every complete (newline-terminated) line, keeping any trailing
        // partial line in the buffer for the next call. The newest parsed fix wins.
        std::size_t newline = 0;
        while ((newline = buffer_.find('\n')) != std::string::npos) {
            const std::string line = buffer_.substr(0, newline);
            buffer_.erase(0, newline + 1);
            if (const std::optional<GnssFix> fix = ParseGgaSentence(line); fix.has_value()) {
                latest_ = *fix;
            }
        }

        return latest_;
    }

}} // namespace TERCOM::Sensors

#else

namespace TERCOM { namespace Sensors {

    SerialNmeaGnss::SerialNmeaGnss(const std::string&, unsigned) {
        throw std::runtime_error("SerialNmeaGnss is only supported on POSIX platforms");
    }

    SerialNmeaGnss::~SerialNmeaGnss() = default;

    GnssFix SerialNmeaGnss::Read() {
        throw std::runtime_error("SerialNmeaGnss is only supported on POSIX platforms");
    }

}} // namespace TERCOM::Sensors

#endif
