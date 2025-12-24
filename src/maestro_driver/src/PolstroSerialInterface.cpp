#include <cassert>
#include "PolstroSerialInterface.h"
#include "PolstroSerialInterfacePOSIX.h"

namespace Polstro {

SerialInterface::SerialInterface(const std::string& portName) {}
SerialInterface::~SerialInterface() {}

bool SerialInterface::setTarget(unsigned char channelNumber, unsigned short target) {
    if (!isOpen()) return false;
    assert(target >= getMinChannelValue() && target <= getMaxChannelValue());
    unsigned char command[4] = {0x84, channelNumber, static_cast<unsigned char>(target & 0x7F), 
                                static_cast<unsigned char>((target >> 7) & 0x7F)};
    return writeBytes(command, sizeof(command));
}

bool SerialInterface::setTargetMSS(unsigned char miniSCCChannelNumber, unsigned char normalizedTarget) {
    if (!isOpen()) return false;
    assert(normalizedTarget >= 0 && normalizedTarget <= 254);
    unsigned char command[3] = {0xFF, miniSCCChannelNumber, normalizedTarget};
    return writeBytes(command, sizeof(command));
}

bool SerialInterface::setSpeed(unsigned char channelNumber, unsigned short speed) {
    if (!isOpen()) return false;
    unsigned char command[4] = {0x87, channelNumber, static_cast<unsigned char>(speed & 0x7F), 
                                static_cast<unsigned char>((speed >> 7) & 0x7F)};
    return writeBytes(command, sizeof(command));
}

bool SerialInterface::setAcceleration(unsigned char channelNumber, unsigned char acceleration) {
    if (!isOpen()) return false;
    unsigned short accelerationAsShort = acceleration;
    unsigned char command[4] = {0x89, channelNumber, static_cast<unsigned char>(accelerationAsShort & 0x7F), 
                                static_cast<unsigned char>((accelerationAsShort >> 7) & 0x7F)};
    return writeBytes(command, sizeof(command));
}

bool SerialInterface::getPosition(unsigned char channelNumber, unsigned short& position) {
    if (!isOpen()) return false;
    position = 0;
    unsigned char command[2] = {0x90, channelNumber};
    if (!writeBytes(command, sizeof(command))) return false;
    unsigned char response[2] = {0x00, 0x00};
    if (!readBytes(response, sizeof(response))) return false;
    position = response[0] + 256 * response[1];
    return true;
}

bool SerialInterface::getMovingState(bool& servosAreMoving) {
    if (!isOpen()) return false;
    servosAreMoving = false;
    unsigned char command = 0x93;
    if (!writeBytes(&command, sizeof(command))) return false;
    unsigned char response = 0x00;
    if (!readBytes(&response, sizeof(response))) return false;
    if (response != 0x00 && response != 0x01) return false;
    servosAreMoving = (response == 0x01);
    return true;
}

bool SerialInterface::getErrors(unsigned short& errors) {
    if (!isOpen()) return false;
    unsigned char command = 0xA1;
    if (!writeBytes(&command, sizeof(command))) return false;
    unsigned char response[2] = {0x00, 0x00};
    if (!readBytes(response, sizeof(response))) return false;
    errors = (response[0] & 0x7F) + 256 * (response[1] & 0x7F);
    return true;
}

bool SerialInterface::goHome() {
    if (!isOpen()) return false;
    unsigned char command = 0xA2;
    return writeBytes(&command, sizeof(command));
}

SerialInterface* SerialInterface::createSerialInterface(const std::string& portName, unsigned int baudRate) {
    return new SerialInterfacePOSIX(portName, baudRate);
}

}
