#include "ulixarm_cpp/serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>


SerialPort::SerialPort()
{
    return;
}

SerialPort::~SerialPort()
{
    close(this->fd_);
    return;
}

int SerialPort::init(const char* port, speed_t baudrate, time_t timeout_ms)
{
    // open serial port
    int fd = this->open_serial_port(port);
    if (fd < 0)
        return -1;
    
    // configure sertial port
    int rc = this->configure_serial_port(fd, baudrate);
    if (rc < 0)
        return rc;

    this->fd_ = fd;

    // timeout
    this->timeout_ms_ = timeout_ms;

    return 1;
}

int SerialPort::write(const uint8_t* data, size_t len)
{
    int rc;
    
    rc = static_cast<int>(::write(this->fd_, data, len));
    if (rc < 0)
        return rc;
    
    return 1;
}


int SerialPort::read(uint8_t* data, uint8_t head, uint8_t end, ssize_t len)
{
    // check if port still connected
    if (this->fd_ < 0)
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

    // check if last byte is end
    if (data[bytes_read - 1] != end)
        return -1;
        
    return 1;
}


int SerialPort::open_serial_port(const char* port)
{
    int fd = open(port, O_RDWR | O_NOCTTY);
    return fd;
}


int SerialPort::configure_serial_port(int fd, speed_t baudrate)
{
    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0)
        return -1;

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
    tty.c_cflag |= CREAD;     // enable receiver
    tty.c_cflag |= CLOCAL;    // ignore modem control lines

    tty.c_iflag = 0;          // raw input
    tty.c_oflag = 0;          // raw output
    tty.c_lflag = 0;          // raw mode, no echo, no signals

    tty.c_cc[VMIN]  = 0;      // non-blocking read
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        return -1;

    return 1;
}


int SerialPort::wait_for_byte(uint8_t* out)
{
    FD_ZERO(&this->read_fds_);
    FD_SET(this->fd_, &this->read_fds_);

    this->timeout_.tv_sec = this->timeout_ms_ / 1000;
    this->timeout_.tv_usec = (this->timeout_ms_ % 1000) * 1000;

    int ret = select(this->fd_ + 1, &this->read_fds_, nullptr, nullptr, &this->timeout_);
    if (ret <= 0)
        ret;

    ssize_t n = ::read(this->fd_, out, 1);
    if (n != 1)
        -1;
        
    return 1;
}