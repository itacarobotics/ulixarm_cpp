#pragma once

#include <libserialport.h>
#include <sys/types.h>

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <exception>

class SerialPort
{
public:
    SerialPort() = default;

    ~SerialPort()
    {
        if (this->port_ != nullptr)
        {
            sp_close(this->port_);
            sp_free_port(this->port_);
        }
    }

    void init(const char* port, unsigned int baudrate, unsigned int timeout_ms)
    {
        if (port == nullptr)
            throw std::invalid_argument("Serial port name cannot be null.");

        if (this->port_ != nullptr)
            throw std::runtime_error("Serial port is already initialized.");

        struct sp_port* sp = nullptr;

        try
        {
            this->open_serial_port(port, &sp);
        }
        catch (...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to open serial port.")
            );
        }

        try
        {
            this->configure_serial_port(sp, baudrate);
        }
        catch (...)
        {
            sp_close(sp);
            sp_free_port(sp);

            std::throw_with_nested(
                std::runtime_error("Failed to configure serial port.")
            );
        }

        this->port_ = sp;
        this->timeout_ms_ = timeout_ms;
    }

    void write(const uint8_t* data, size_t len)
    {
        if (this->port_ == nullptr)
            throw std::runtime_error("Port is not open.");

        if (data == nullptr && len > 0)
            throw std::invalid_argument("Write buffer cannot be null.");

        const int rc = sp_blocking_write(
            this->port_,
            data,
            len,
            this->timeout_ms_
        );

        if (rc < 0)
        {
            throw std::runtime_error(
                "Failed to write to serial port."
            );
        }

        if (static_cast<size_t>(rc) < len)
        {
            throw std::runtime_error(
                "Serial port write timed out."
            );
        }
    }

    void read(uint8_t* data, uint8_t head, uint8_t tail, ssize_t len)
    {
        if (this->port_ == nullptr)
            throw std::runtime_error("Port is not open.");

        if (len <= 0)
            throw std::invalid_argument(
                "Read length must be greater than zero."
            );

        if (data == nullptr)
            throw std::invalid_argument("Read buffer cannot be null.");

        int rc;
        uint8_t byte;
        ssize_t bytes_read = 0;

        // Read bytes until the header is found.
        while (true)
        {
            this->wait_for_byte(&byte);

            if (byte == head)
                break;
        }

        data[bytes_read] = head;
        bytes_read += 1;

        // Read the remaining bytes.
        while (bytes_read < len)
        {
            this->wait_for_byte(&byte);

            data[bytes_read] = byte;
            bytes_read += 1;
        }

        // Check whether the last byte matches the expected tail.
        if (data[bytes_read - 1] != tail)
        {
            throw std::runtime_error(
                "Invalid serial frame: unexpected tail byte."
            );
        }
    }

private:
    void open_serial_port(const char* port, struct sp_port** out_port)
    {
        if (sp_get_port_by_name(port, out_port) != SP_OK)
        {
            throw std::runtime_error(
                std::string("Could not get serial port by name: ") + port
            );
        }

        if (sp_open(*out_port, SP_MODE_READ_WRITE) != SP_OK)
        {
            sp_free_port(*out_port);
            *out_port = nullptr;

            throw std::runtime_error(
                std::string("Could not open serial port: ") + port
            );
        }
    }

    void configure_serial_port(struct sp_port* port, unsigned int baudrate)
    {
        if (sp_set_baudrate(port, static_cast<int>(baudrate)) != SP_OK)
        {
            throw std::runtime_error("Could not set baudrate.");
        }

        if (sp_set_bits(port, 8) != SP_OK)
        {
            throw std::runtime_error("Could not set data bits.");
        }

        if (sp_set_parity(port, SP_PARITY_NONE) != SP_OK)
        {
            throw std::runtime_error("Could not set parity.");
        }

        if (sp_set_stopbits(port, 1) != SP_OK)
        {
            throw std::runtime_error("Could not set stop bits.");
        }

        if (sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE) != SP_OK)
        {
            throw std::runtime_error("Could not set flow control.");
        }
    }

    void wait_for_byte(uint8_t* out)
    {
        const int rc = sp_blocking_read(this->port_, out, 1, this->timeout_ms_);

        if (rc < 0)
        {
            throw std::runtime_error(
                "Failed to read from serial port."
            );
        }

        if (rc == 0)
        {
            throw std::runtime_error(
                "Serial port read timed out."
            );
        }
    }

    struct sp_port* port_ = nullptr;
    unsigned int timeout_ms_ = 0;
};