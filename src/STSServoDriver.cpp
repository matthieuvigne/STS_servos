#include "STSServoDriver.h"
#include "HardwareSerial.h"

namespace instruction
{
    byte const PING_       = 0x01;
    byte const READ       = 0x02;
    byte const WRITE      = 0x03;
    byte const REGWRITE   = 0x04;
    byte const ACTION     = 0x05;
    byte const SYNCWRITE  = 0x83;
    byte const RESET      = 0x06;
};

STSServoDriver::STSServoDriver() : dirPin_(0)
{
}

bool STSServoDriver::init(byte const& dirPin, HardwareSerial *serialPort,long const& baudRate)
{
#if defined(SERIAL_H) || defined(HardwareSerial_h)
    if (serialPort == nullptr)
        serialPort = &Serial;
#endif
    // Open port
    port_ = serialPort;
    port_->begin(baudRate);
    port_->setTimeout(2);
    dirPin_ = dirPin;
    if (this->dirPin_ < 255)
    {
        pinMode(dirPin_, OUTPUT);
    }

    for (int i = 0; i < 256; i++)
        servoType_[i] = ServoType::UNKNOWN;

    // Test that a servo is present.
    for (byte i = 0; i < 0xFE; i++)
        if (ping(i))
            return true;
    return false;
}

bool STSServoDriver::init(HardwareSerial *serialPort, long const& baudRate)
{
    return this->init(255, serialPort, baudRate);
}

bool STSServoDriver::ping(byte const &servoId)
{
    byte response[1] = {0xFF};
    int send = sendMessage(servoId,
                           instruction::PING_,
                           0,
                           response);
    // Failed to send
    if (send != 6)
        return false;
    // Read response
    int rd = receiveMessage(servoId, 1, response);
    if (rd < 0)
        return false;
    return response[0] == 0x00;
}

bool STSServoDriver::setId(byte const &oldServoId, byte const &newServoId)
{
    if (servoType_[oldServoId] == ServoType::UNKNOWN)
    {
        determineServoType(oldServoId);
    }

    if (oldServoId >= 0xFE || newServoId >= 0xFE)
        return false;
    if (ping(newServoId))
        return false; // address taken

    unsigned char lockRegister = STSRegisters::WRITE_LOCK;
    if (servoType_[oldServoId] == ServoType::SCS)
    {
        lockRegister = STSRegisters::TORQUE_LIMIT; // On SCS, this has been remapped.
    }
    // Unlock EEPROM
    if (!writeRegister(oldServoId, lockRegister, 0))
        return false;
    delay(5);
    // Write new ID
    if (!writeRegister(oldServoId, STSRegisters::ID, newServoId))
        return false;
    // Lock EEPROM
    delay(5);
    if (!writeRegister(newServoId, lockRegister, 1))
        return false;
    // Give it some time to change id.
    bool hasPing = false;
    int nIter = 0;
    while (!hasPing && nIter < 10)
    {
      delay(50);
      hasPing = ping(newServoId);
    }
    if (hasPing)
    {
      // Update servo type cache.
      servoType_[newServoId] = servoType_[oldServoId];
      servoType_[oldServoId] = ServoType::UNKNOWN;
    }
    return hasPing;
}

bool STSServoDriver::setPositionOffset(byte const &servoId, int const &positionOffset)
{
    if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 0))
        return false;
    // Write new position offset
    if (!writeTwoBytesRegister(servoId, STSRegisters::POSITION_CORRECTION, positionOffset))
        return false;
    // Lock EEPROM
    if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 1))
        return false;
    return true;
}

int STSServoDriver::getCurrentPosition(byte const &servoId)
{
    return readTwoBytesRegister(servoId, STSRegisters::CURRENT_POSITION);
}

int STSServoDriver::getCurrentSpeed(byte const &servoId)
{
    return readTwoBytesRegister(servoId, STSRegisters::CURRENT_SPEED);
}

int STSServoDriver::getCurrentTemperature(byte const &servoId)
{
    return readTwoBytesRegister(servoId, STSRegisters::CURRENT_TEMPERATURE);
}

float STSServoDriver::getCurrentCurrent(byte const &servoId)
{
    int16_t current = readTwoBytesRegister(servoId, STSRegisters::CURRENT_CURRENT);
    return current * 0.0065;
}

bool STSServoDriver::isMoving(byte const &servoId)
{
    byte const result = readRegister(servoId, STSRegisters::MOVING_STATUS);
    return result > 0;
}

bool STSServoDriver::setTargetPosition(byte const &servoId, int const &position, int const &speed, bool const &asynchronous)
{
    byte params[6] = {0, 0, // Position
        0, 0, // Padding
        0, 0}; // Velocity
    convertIntToBytes(servoId, position, &params[0]);
    convertIntToBytes(servoId, speed, &params[4]);
    return writeRegisters(servoId, STSRegisters::TARGET_POSITION, sizeof(params), params, asynchronous);
}

bool STSServoDriver::setTargetVelocity(byte const &servoId, int const &velocity, bool const &asynchronous)
{
    return writeTwoBytesRegister(servoId, STSRegisters::RUNNING_SPEED, velocity, asynchronous);
}

bool STSServoDriver::setTargetAcceleration(byte const &servoId, byte const &acceleration, bool const &asynchronous)
{
    return writeRegister(servoId, STSRegisters::TARGET_ACCELERATION, acceleration, asynchronous);
}

bool STSServoDriver::setMode(unsigned char const& servoId, STSMode const& mode)
{
    return writeRegister(servoId, STSRegisters::OPERATION_MODE, static_cast<unsigned char>(mode));
}


bool STSServoDriver::trigerAction()
{
    byte noParam = 0;
    int send = sendMessage(0xFE, instruction::ACTION, 0, &noParam);
    return send == 6;
}

int STSServoDriver::sendMessage(byte const &servoId,
                                byte const &commandID,
                                byte const &paramLength,
                                byte *parameters)
{
    byte message[6 + paramLength];
    byte checksum = servoId + paramLength + 2 + commandID;
    message[0] = 0xFF;
    message[1] = 0xFF;
    message[2] = servoId;
    message[3] = paramLength + 2;
    message[4] = commandID;
    for (int i = 0; i < paramLength; i++)
    {
        message[5 + i] = parameters[i];
        checksum += parameters[i];
    }
    message[5 + paramLength] = ~checksum;
    if (this->dirPin_ < 255){
        digitalWrite(dirPin_, HIGH);
    }
    int ret = port_->write(message, 6 + paramLength);
    if (this->dirPin_ < 255){
        digitalWrite(dirPin_, LOW);
    }
    // Give time for the message to be processed.
    delayMicroseconds(200);
    return ret;
}

bool STSServoDriver::writeRegisters(byte const &servoId,
                                    byte const &startRegister,
                                    byte const &writeLength,
                                    byte const *parameters,
                                    bool const &asynchronous)
{
    byte param[writeLength + 1];
    param[0] = startRegister;
    for (int i = 0; i < writeLength; i++)
        param[i + 1] = parameters[i];
    int rc = sendMessage(servoId,
                         asynchronous ? instruction::REGWRITE : instruction::WRITE,
                         writeLength + 1,
                         param);
    return rc == writeLength + 7;
}

bool STSServoDriver::writeRegister(byte const &servoId,
                                   byte const &registerId,
                                   byte const &value,
                                   bool const &asynchronous)
{
    return writeRegisters(servoId, registerId, 1, &value, asynchronous);
}

bool STSServoDriver::writeTwoBytesRegister(byte const &servoId,
                                           byte const &registerId,
                                           int16_t const &value,
                                           bool const &asynchronous)
{
    byte params[2] = {0, 0};
    convertIntToBytes(servoId, value, params);
    return writeRegisters(servoId, registerId, 2, params, asynchronous);
}

byte STSServoDriver::readRegister(byte const &servoId, byte const &registerId)
{
    byte result = 0;
    int rc = readRegisters(servoId, registerId, 1, &result);
    if (rc < 0)
        return 0;
    return result;
}

int16_t STSServoDriver::readTwoBytesRegister(byte const &servoId, byte const &registerId)
{
    if (servoType_[servoId] == ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    unsigned char result[2] = {0, 0};
    int16_t value = 0;
    int16_t signedValue = 0;
    int rc = readRegisters(servoId, registerId, 2, result);
    if (rc < 0)
        return 0;
    switch(servoType_[servoId])
    {
        case ServoType::SCS:
            value = static_cast<int16_t>(result[1] +  (result[0] << 8));
            // Bit 15 is sign
            signedValue = value & ~0x8000;
            if (value & 0x8000)
                signedValue = -signedValue;
            return signedValue;
        case ServoType::STS:
            value = static_cast<int16_t>(result[0] +  (result[1] << 8));
            // Bit 15 is sign
            signedValue = value & ~0x8000;
            if (value & 0x8000)
                signedValue = -signedValue;
            return signedValue;
        default:
            return 0;
    }
}

int STSServoDriver::readRegisters(byte const &servoId,
                                  byte const &startRegister,
                                  byte const &readLength,
                                  byte *outputBuffer)
{
    byte readParam[2] = {startRegister, readLength};
    // Flush
    while (port_->read() != -1)
        ;;
    int send = sendMessage(servoId, instruction::READ, 2, readParam);
    // Failed to send
    if (send != 8)
        return -1;
    // Read
    byte result[readLength + 1];
    int rd = receiveMessage(servoId, readLength + 1, result);
    if (rd < 0)
        return rd;

    for (int i = 0; i < readLength; i++)
        outputBuffer[i] = result[i + 1];
    return 0;
}

int STSServoDriver::receiveMessage(byte const& servoId,
                                   byte const& readLength,
                                   byte *outputBuffer)
{
    if (this->dirPin_ < 255){
        digitalWrite(dirPin_, LOW);
    }
    
    byte result[readLength + 5];
    size_t rd = port_->readBytes(result, readLength + 5);
    if (rd != (unsigned short)(readLength + 5))
        return -1;
    // Check message integrity
    if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servoId || result[3] != readLength + 1)
        return -2;
    byte checksum = 0;
    for (int i = 2; i < readLength + 4; i++)
        checksum += result[i];
    checksum = ~checksum;
    if (result[readLength + 4] != checksum)
        return -3;

    // Copy result to output buffer
    for (int i = 0; i < readLength; i++)
        outputBuffer[i] = result[i + 4];
    return 0;
}

void STSServoDriver::convertIntToBytes(byte const& servoId, int const &value, byte result[2])
{
    uint16_t servoValue = 0;
    if (servoType_[servoId] == ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    // Handle different servo type.
    switch(servoType_[servoId])
    {
        case ServoType::SCS:
            // Little endian ; byte 10 is sign.
            servoValue = abs(value);
            if (value < 0)
                servoValue = 0x0400  | servoValue;
            // Invert endianness
            servoValue = (servoValue >> 8) + ((servoValue & 0xFF) << 8);
            break;
        case ServoType::STS:
        default:
            servoValue = abs(value);
            if (value < 0)
                servoValue = 0x8000  | servoValue;
            break;
    }
    result[0] = static_cast<unsigned char>(servoValue & 0xFF);
    result[1] = static_cast<unsigned char>((servoValue >> 8) & 0xFF);
}

void STSServoDriver::sendAndUpdateChecksum(byte convertedValue[], byte &checksum)
{
    port_->write(convertedValue, 2);
    checksum += convertedValue[0] + convertedValue[1];
}

void STSServoDriver::setTargetPositions(byte const &numberOfServos, const byte servoIds[],
                                        const int positions[],
                                        const int speeds[])
{
    port_->write(0xFF);
    port_->write(0xFF);
    port_->write(0XFE);
    port_->write(numberOfServos * 7 + 4);
    port_->write(instruction::SYNCWRITE);
    port_->write(STSRegisters::TARGET_POSITION);
    port_->write(6);
    byte checksum = 0xFE + numberOfServos * 7 + 4 + instruction::SYNCWRITE + STSRegisters::TARGET_POSITION + 6;
    for (int index = 0; index < numberOfServos; index++)
    {
        checksum += servoIds[index];
        port_->write(servoIds[index]);
        byte intAsByte[2];
        convertIntToBytes(servoIds[index], positions[index], intAsByte);
        sendAndUpdateChecksum(intAsByte, checksum);
        port_->write(0);
        port_->write(0);
        convertIntToBytes(servoIds[index], speeds[index], intAsByte);
        sendAndUpdateChecksum(intAsByte, checksum);
    }
    port_->write(~checksum);
}

void STSServoDriver::determineServoType(byte const& servoId)
{
    switch(readRegister(servoId, STSRegisters::SERVO_MAJOR))
    {
        case 9: servoType_[servoId] = ServoType::STS; break;
        case 5: servoType_[servoId] = ServoType::SCS; break;
    }
}
