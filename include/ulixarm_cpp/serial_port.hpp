#pragma once

#include <libserialport.h>
#include <sys/types.h>

#include <cstdint>
#include <cstddef>
#include <ctime>

#include <stdexcept>

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
        return;
    }

    void init(const char* port, uint baudrate, uint timeout_ms)
    {
        // open serial port
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
        
        // configure serial port
        try
        {
            this->configure_serial_port(sp, baudrate);
        }
        catch(...)
        {
            std::throw_with_nested(
                std::runtime_error("Failed to configure serial port.")
            );
        }
        
        this->port_ = sp;
        this->timeout_ms_ = timeout_ms;

        return;
    }

    void write(const uint8_t* data, size_t len)
    {
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

    int read(uint8_t* data, uint8_t head, uint8_t tail, ssize_t len)
    {
        // check if port still connected
        if (this->port_ == nullptr)
            return -1;
    
        int rc;
        uint8_t byte;
        ssize_t bytes_read = 0;

        // read bytes until head is found
        while (true)
        {
            rc = wait_for_byte(&byte);
            if (rc < 0)
                return rc;
            if (byte == head)
                break;
        }
        
        data[bytes_read] = head;
        bytes_read += 1;

        // read len - 1 remaining bytes
        while (bytes_read < len)
        {
            rc = wait_for_byte(&byte);
            if (rc < 0)
                return rc;
            data[bytes_read] = byte;
            bytes_read += 1;
        }

            // check if last byte is tail
        if (data[bytes_read - 1] != tail)
            return -1;
        
        return 1;
    }

private:
    void open_serial_port(const char* port, struct sp_port** out_port)
    {
        if (sp_get_port_by_name(port, out_port) != SP_OK)
        {
            throw std::runtime_error("Could not get port by name, tried with " + *port);
        }
            
        if (sp_open(*out_port, SP_MODE_READ_WRITE) != SP_OK)
        {
            sp_free_port(*out_port);
            throw std::runtime_error("Could not open seril port " + *port);
        }
    }


    void configure_serial_port(struct sp_port* port, uint baudrate)
    {
        if (sp_set_baudrate(port, baudrate) != SP_OK)
        {
            throw std::runtime_error("Could not set baudrate.");
            return;
        }

        if (sp_set_bits(port, 8) != SP_OK)
        {
            throw std::runtime_error("Could not set bits.");
            return;
        }

        if (sp_set_parity(port, SP_PARITY_NONE) != SP_OK)
        {
            throw std::runtime_error("Could not set parity.");
            return;
        }

        if (sp_set_stopbits(port, 1) != SP_OK)
        {
            throw std::runtime_error("Could not set stop bits.");
            return;
        }

        if (sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE) != SP_OK)
        {
            throw std::runtime_error("Could not set flow control.");
            return;
        }

        return;
    }


    void wait_for_byte(uint8_t* out)
    {
        int rc = sp_blocking_read(this->port_, out, 1, this->timeout_ms_);
        if (rc < 0)
            throw std::runtime_error("Failed to read from serial port.");

        if (rc == 0)
            throw std::runtime_error("Serial port read timed out.");
    }

    struct sp_port* port_ = nullptr;
    uint timeout_ms_ = 0;
};