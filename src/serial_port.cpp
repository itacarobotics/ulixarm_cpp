#include "serial_port.hpp"

#include <termios.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>


namespace asio = boost::asio;


namespace
{

std::string to_hex(uint8_t value)
{
    char buf[5];
    std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<unsigned>(value));
    return std::string(buf);
}


// Starts an asynchronous operation via `start(handler)` and runs the io_context
// until it completes or `timeout` expires. On timeout the operation is
// cancelled and the aborted handler is drained before returning, so no handler
// is left referencing this function's locals.
//
// Returns the number of bytes transferred.
template <typename StartFn>
std::size_t run_with_timeout(
    asio::io_context& io,
    asio::serial_port& port,
    std::chrono::milliseconds timeout,
    StartFn&& start,
    const char* what)
{
    boost::system::error_code ec;
    std::size_t transferred = 0;
    bool done = false;

    io.restart();

    start(
        [&](const boost::system::error_code& e, std::size_t n)
        {
            ec = e;
            transferred = n;
            done = true;
        }
    );

    io.run_for(timeout);

    const bool timed_out = !done;

    if (timed_out)
    {
        boost::system::error_code ignored;
        port.cancel(ignored);

        // Let the (aborted) completion handler run.
        io.restart();
        io.run();
    }

    if (!done)
    {
        throw std::runtime_error(
            std::string(what) + " failed: operation did not complete."
        );
    }

    if (timed_out && ec == asio::error::operation_aborted)
    {
        throw std::runtime_error(
            std::string(what) + " timeout: no data within " +
            std::to_string(timeout.count()) + " ms."
        );
    }

    if (ec)
    {
        throw std::runtime_error(
            std::string(what) + " failed: " + ec.message()
        );
    }

    return transferred;
}

} // namespace


SerialPort::SerialPort()
    : io_(),
      port_(io_),
      rx_buf_(),
      timeout_ms_(0)
{
}


void SerialPort::init(const char* port, unsigned int baudrate, time_t timeout_ms)
{
    if (port == nullptr)
    {
        throw std::invalid_argument(
            "Serial port initialization failed: port is null."
        );
    }

    boost::system::error_code ignored;

    // If the port is already open, close it before reinitializing.
    port_.close(ignored);
    rx_buf_.consume(rx_buf_.size());

    try
    {
        port_.open(port);
        this->configure_serial_port(baudrate);

        // Store configuration only after everything succeeded.
        this->timeout_ms_ = timeout_ms;
    }
    catch (...)
    {
        port_.close(ignored);

        std::throw_with_nested(
            std::runtime_error(
                "Failed to initialize serial port '" + std::string(port) + "'."
            )
        );
    }
}


void SerialPort::write(const uint8_t* data, size_t len)
{
    if (!port_.is_open())
    {
        throw std::runtime_error(
            "Serial write failed: serial port is not open."
        );
    }

    if (data == nullptr)
    {
        throw std::invalid_argument(
            "Serial write failed: data pointer is null."
        );
    }

    if (len == 0)
    {
        return;
    }

    try
    {
        // async_write only completes successfully once all bytes are written.
        run_with_timeout(
            io_,
            port_,
            std::chrono::milliseconds(timeout_ms_),
            [&](auto&& handler)
            {
                asio::async_write(port_, asio::buffer(data, len), handler);
            },
            "Serial write"
        );
    }
    catch (...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to write data to serial port.")
        );
    }
}


void SerialPort::read(
    uint8_t* data,
    uint8_t head,
    uint8_t tail,
    ssize_t len)
{
    if (!port_.is_open())
    {
        throw std::runtime_error(
            "Serial read failed: serial port is not open."
        );
    }

    if (data == nullptr)
    {
        throw std::invalid_argument(
            "Serial read failed: data pointer is null."
        );
    }

    if (len <= 0)
    {
        throw std::invalid_argument(
            "Serial read failed: requested packet length must be greater than zero."
        );
    }

    try
    {
        const std::size_t total = static_cast<std::size_t>(len);
        const auto timeout = std::chrono::milliseconds(timeout_ms_);

        // Phase 1: synchronize to the header byte.
        // async_read_until stops at the first `head`; anything received after it
        // stays in rx_buf_ and is used by the next phase.
        try
        {
            const std::size_t n = run_with_timeout(
                io_,
                port_,
                timeout,
                [&](auto&& handler)
                {
                    asio::async_read_until(
                        port_, rx_buf_, static_cast<char>(head), handler
                    );
                },
                "Waiting for packet header"
            );

            // Drop the garbage in front of the header; head is now at index 0.
            rx_buf_.consume(n - 1);
        }
        catch (...)
        {
            rx_buf_.consume(rx_buf_.size());
            throw;
        }

        // Phase 2: read the remaining len-1 bytes (minus whatever we already
        // have buffered), then check the tail.
        try
        {
            if (rx_buf_.size() < total)
            {
                const std::size_t missing = total - rx_buf_.size();

                run_with_timeout(
                    io_,
                    port_,
                    timeout,
                    [&](auto&& handler)
                    {
                        asio::async_read(
                            port_,
                            rx_buf_,
                            asio::transfer_exactly(missing),
                            handler
                        );
                    },
                    "Reading packet body"
                );
            }

            asio::buffer_copy(asio::buffer(data, total), rx_buf_.data());

            if (data[total - 1] != tail)
            {
                throw std::runtime_error(
                    "Invalid serial packet: expected end byte " +
                    to_hex(tail) +
                    ", but received " +
                    to_hex(data[total - 1]) +
                    "."
                );
            }

            rx_buf_.consume(total);
        }
        catch (...)
        {
            // Skip only the false/incomplete header so the next read() can
            // re-synchronize on a real header inside the buffered bytes.
            rx_buf_.consume(1);
            throw;
        }
    }
    catch (...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to read packet from serial port.")
        );
    }
}


void SerialPort::configure_serial_port(unsigned int baudrate)
{
    if (!port_.is_open())
    {
        throw std::runtime_error(
            "Failed to configure serial port: port is not open."
        );
    }

    try
    {
        // asio::serial_port::open() already puts the tty in raw mode
        // (cfmakeraw) and sets CREAD | CLOCAL.
        port_.set_option(asio::serial_port_base::baud_rate(baudrate));
        port_.set_option(asio::serial_port_base::character_size(8));
        port_.set_option(asio::serial_port_base::parity(
            asio::serial_port_base::parity::none));
        port_.set_option(asio::serial_port_base::stop_bits(
            asio::serial_port_base::stop_bits::one));
        port_.set_option(asio::serial_port_base::flow_control(
            asio::serial_port_base::flow_control::none));

        // Flush any pending input.
        if (::tcflush(port_.native_handle(), TCIFLUSH) != 0)
        {
            throw std::runtime_error(
                "tcflush() failed: " +
                std::string(std::strerror(errno))
            );
        }
    }
    catch (...)
    {
        std::throw_with_nested(
            std::runtime_error("Failed to configure serial port.")
        );
    }
}