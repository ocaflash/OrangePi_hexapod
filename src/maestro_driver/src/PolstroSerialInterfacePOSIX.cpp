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
