#ifndef ULIXARM_CPP_SERIAL_PORT_HPP
#define ULIXARM_CPP_SERIAL_PORT_HPP

#include <boost/asio.hpp>

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <sys/types.h>

class SerialPort
{
public:
    SerialPort();
    ~SerialPort() = default;

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // baudrate is the numeric rate (e.g. 115200), NOT a termios Bxxxx constant.
    void init(const char* port, unsigned int baudrate, time_t timeout_ms);

    void write(const uint8_t* data, size_t len);

    // Synchronizes to `head`, reads a packet of `len` bytes total
    // (including head and tail) and verifies that the last byte is `tail`.
    void read(uint8_t* data, uint8_t head, uint8_t tail, ssize_t len);

private:
    void configure_serial_port(unsigned int baudrate);

    boost::asio::io_context io_;        // must be declared before port_
    boost::asio::serial_port port_;
    boost::asio::streambuf rx_buf_;     // keeps bytes received past the current packet
    time_t timeout_ms_;
};

#endif // ULIXARM_CPP_SERIAL_PORT_HPP
