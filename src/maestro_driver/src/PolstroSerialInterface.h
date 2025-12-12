#ifndef POLSTROSERIALINTERFACE_H_
#define POLSTROSERIALINTERFACE_H_

#include <string>

namespace Polstro {

class SerialInterface {
public:
    SerialInterface(const std::string& portName);
    virtual ~SerialInterface();

    virtual bool isOpen() const = 0;

    static unsigned int getMinChannelValue() { return mMinChannelValue; }
    static unsigned int getMaxChannelValue() { return mMaxChannelValue; }

    bool setTarget(unsigned char channelNumber, unsigned short target);
    bool setTargetMSS(unsigned char miniSCCChannelNumber, unsigned char normalizedTarget);
    bool setSpeed(unsigned char channelNumber, unsigned short speed);
    bool setAcceleration(unsigned char channelNumber, unsigned char acceleration);
    bool getPosition(unsigned char channelNumber, unsigned short& position);
    bool getMovingState(bool& servosAreMoving);
    bool getErrors(unsigned short& error);
    bool goHome();

    static SerialInterface* createSerialInterface(const std::string& portName, unsigned int baudRate);

private:
    static const unsigned int mMinChannelValue = 2000;
    static const unsigned int mMaxChannelValue = 10000;

    virtual bool writeBytes(const unsigned char* data, unsigned int dataSizeInBytes) = 0;
    virtual bool readBytes(unsigned char* data, unsigned int dataSizeInBytes) = 0;
};

}

#endif
