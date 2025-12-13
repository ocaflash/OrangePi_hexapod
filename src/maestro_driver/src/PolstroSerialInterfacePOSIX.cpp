#include "PolstroSerialInterfacePOSIX.h"

#include <fcntl.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <cassert>
#include <cerrno>

namespace Polstro {

SerialInterfacePOSIX::SerialInterfacePOSIX(const std::string& portName)
    : SerialInterface(portName), mFileDescriptor(-1) {
    mFileDescriptor = openPort(portName);
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

int SerialInterfacePOSIX::openPort(const std::string& portName) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror(portName.c_str());
        return -1;
    }
    
    // Configure serial port
    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate to 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
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
    
    tcsetattr(fd, TCSANOW, &options);
    
    return fd;
}

bool SerialInterfacePOSIX::writeBytes(const unsigned char* data, unsigned int numBytesToWrite) {
    assert(isOpen());
    ssize_t ret = write(mFileDescriptor, data, numBytesToWrite);
    if (ret == -1) {
        printf("Error writing. errno=%d\n", errno);
        return false;
    }
    assert(ret == static_cast<ssize_t>(numBytesToWrite));
    return true;
}

bool SerialInterfacePOSIX::readBytes(unsigned char* data, unsigned int numBytesToRead) {
    assert(isOpen());
    ssize_t ret = read(mFileDescriptor, data, numBytesToRead);
    if (ret == -1) {
        printf("Error reading. errno=%d\n", errno);
        return false;
    }
    assert(ret == static_cast<ssize_t>(numBytesToRead));
    return true;
}

}
