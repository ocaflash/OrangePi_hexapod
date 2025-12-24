#include "PolstroSerialInterfacePOSIX.h"

#include <fcntl.h>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <cassert>
#include <cerrno>

namespace Polstro {

namespace {
speed_t baudRateToSpeedT(unsigned int baudRate) {
    switch (baudRate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}
}  // namespace

SerialInterfacePOSIX::SerialInterfacePOSIX(const std::string& portName, unsigned int baudRate)
    : SerialInterface(portName), mFileDescriptor(-1) {
    mFileDescriptor = openPort(portName, baudRate);
}

SerialInterfacePOSIX::~SerialInterfacePOSIX() {
    if (isOpen()) {
        goHome();
        close(mFileDescriptor);
    }
    mFileDescriptor = -1;
}

bool SerialInterfacePOSIX::isOpen() const {
    return mFileDescriptor != -1;
}

int SerialInterfacePOSIX::openPort(const std::string& portName, unsigned int baudRate) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror(portName.c_str());
        return -1;
    }
    
    // Configure serial port
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        std::fprintf(stderr, "tcgetattr() failed for %s: %s (errno=%d)\n", portName.c_str(),
                     std::strerror(errno), errno);
        close(fd);
        return -1;
    }
    
    // Set baud rate
    const speed_t speed = baudRateToSpeedT(baudRate);
    if (baudRate != 9600 && baudRate != 19200 && baudRate != 38400 && baudRate != 57600 &&
        baudRate != 115200) {
        std::fprintf(stderr, "Unsupported baudRate=%u for %s; falling back to 115200\n", baudRate,
                     portName.c_str());
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // 8N1 mode
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits
    
    // Enable receiver, ignore modem control lines
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Raw input/output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::fprintf(stderr, "tcsetattr() failed for %s: %s (errno=%d)\n", portName.c_str(),
                     std::strerror(errno), errno);
        close(fd);
        return -1;
    }
    
    return fd;
}

bool SerialInterfacePOSIX::writeBytes(const unsigned char* data, unsigned int numBytesToWrite) {
    assert(isOpen());
    ssize_t ret = write(mFileDescriptor, data, numBytesToWrite);
    if (ret == -1) {
        std::fprintf(stderr, "Serial write() failed: %s (errno=%d)\n", std::strerror(errno), errno);
        return false;
    }
    assert(ret == static_cast<ssize_t>(numBytesToWrite));
    return true;
}

bool SerialInterfacePOSIX::readBytes(unsigned char* data, unsigned int numBytesToRead) {
    assert(isOpen());
    ssize_t ret = read(mFileDescriptor, data, numBytesToRead);
    if (ret == -1) {
        std::fprintf(stderr, "Serial read() failed: %s (errno=%d)\n", std::strerror(errno), errno);
        return false;
    }
    assert(ret == static_cast<ssize_t>(numBytesToRead));
    return true;
}

}
