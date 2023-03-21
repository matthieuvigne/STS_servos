#include "STSServoDriver.h"

namespace instruction
{
    byte const PING      = 0x01;
    byte const READ      = 0x02;
    byte const WRITE     = 0x03;
    byte const REGWRITE  = 0x04;
    byte const ACTION    = 0x05;
    byte const SYNCWRITE = 0x83;
    byte const RESET     = 0x06;
};


STSServoDriver::STSServoDriver():
    dirPin_(0)
{
}


bool STSServoDriver::init(byte const& dirPin, HardwareSerial *serialPort,long const& baudRate)
{
    #ifdef SERIAL_H
    if (serialPort == nullptr)
        serialPort = &Serial;
    #endif
    // Open port
    port_ = serialPort;
    port_->begin(baudRate);
    dirPin_ = dirPin;
    pinMode(dirPin_, OUTPUT);

    // Test that a servo is present.
    for (byte i = 0; i < 0xFE; i++)
        if (ping(i))
            return true;
    return false;
}


bool STSServoDriver::ping(byte const& servoId)
{
    byte response[1] = {0xFF};
    int send = sendMessage(servoId,
                           instruction::PING,
                           0,
                           response);
    // Failed to send
    if (send != 6)
        return false;
    // Read response
    int rd = recieveMessage(servoId, 1, response);
    if (rd < 0)
        return false;
    return response[0] == 0x00;
}


bool STSServoDriver::setId(byte const& oldServoId, byte const& newServoId)
{
    if (oldServoId >= 0xFE || newServoId >= 0xFE)
        return false;
    if (ping(newServoId))
        return false; // address taken
    // Unlock EEPROM
    if (!writeRegister(oldServoId, STSRegisters::WRITE_LOCK, 0))
        return false;
    // Write new ID
    if (!writeRegister(oldServoId, STSRegisters::ID, newServoId))
        return false;
    // Lock EEPROM
    if (!writeRegister(newServoId, STSRegisters::WRITE_LOCK, 1))
      return false;
    return ping(newServoId);
}


int STSServoDriver::getCurrentPosition(byte const& servoId)
{
    int16_t pos = readTwoBytesRegister(servoId, STSRegisters::CURRENT_POSITION);
    return pos;
}


int STSServoDriver::getCurrentSpeed(byte const& servoId)
{
    int16_t vel = readTwoBytesRegister(servoId, STSRegisters::CURRENT_SPEED);
    return vel;
}


int STSServoDriver::getCurrentTemperature(byte const& servoId)
{
    return readTwoBytesRegister(servoId, STSRegisters::CURRENT_TEMPERATURE);
}


int STSServoDriver::getCurrentCurrent(byte const& servoId)
{
    int16_t current = readTwoBytesRegister(servoId, STSRegisters::CURRENT_CURRENT);
    return current * 0.0065;
}

bool STSServoDriver::isMoving(byte const& servoId)
{
    byte const result = readRegister(servoId, STSRegisters::MOVING_STATUS);
    return result > 0;
}


bool STSServoDriver::setTargetPosition(byte const& servoId, int const& position, bool const& asynchronous)
{
    return writeTwoBytesRegister(servoId, STSRegisters::TARGET_POSITION, position, asynchronous);
}


bool STSServoDriver::setTargetVelocity(byte const& servoId, int const& velocity, bool const& asynchronous)
{
    return writeTwoBytesRegister(servoId, STSRegisters::RUNNING_SPEED, velocity, asynchronous);
}


bool STSServoDriver::trigerAction()
{
    byte noParam = 0;
    int send = sendMessage(0xFE, instruction::ACTION, 0, &noParam);
    return send == 6;
}

int STSServoDriver::sendMessage(byte const& servoId,
                                byte const& commandID,
                                byte const& paramLength,
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

    digitalWrite(dirPin_, HIGH);
    int ret = port_->write(message, 6 + paramLength);
    digitalWrite(dirPin_, LOW);
    // Give time for the message to be processed.
    delayMicroseconds(200);
    return ret;
}


bool STSServoDriver::writeRegisters(byte const& servoId,
                                    byte const& startRegister,
                                    byte const& writeLength,
                                    byte const *parameters,
                                    bool const& asynchronous)
{
    byte param[writeLength + 1];
    param[0] = startRegister;
    for (int i = 0; i < writeLength; i++)
        param[i + 1] = parameters[i];
    int rc =  sendMessage(servoId,
                          asynchronous ? instruction::REGWRITE : instruction::WRITE,
                          writeLength + 1,
                          param);
    return rc == writeLength + 7;
}


bool STSServoDriver::writeRegister(byte const& servoId,
                                   byte const& registerId,
                                   byte const& value,
                                   bool const& asynchronous)
{
    return writeRegisters(servoId, registerId, 1, &value, asynchronous);
}


bool STSServoDriver::writeTwoBytesRegister(byte const& servoId,
                                           byte const& registerId,
                                           int16_t const& value,
                                           bool const& asynchronous)
{
    byte params[2] = {static_cast<unsigned char>(value & 0xFF),
                               static_cast<unsigned char>((value >> 8) & 0xFF)};
    return writeRegisters(servoId, registerId, 2, params, asynchronous);
}


byte STSServoDriver::readRegister(byte const& servoId, byte const& registerId)
{
    byte result = 0;
    int rc = readRegisters(servoId, registerId, 1, &result);
    if (rc < 0)
        return 0;
    return result;
}


int16_t STSServoDriver::readTwoBytesRegister(byte const& servoId, byte const& registerId)
{
    byte result[2] = {0, 0};
    int rc = readRegisters(servoId, registerId, 2, result);
    if (rc < 0)
        return 0;
    return result[0] + (result[1] << 8);
}


int STSServoDriver::readRegisters(byte const& servoId,
                                  byte const& startRegister,
                                  byte const& readLength,
                                  byte *outputBuffer)
{
    byte readParam[2] = {startRegister, readLength};
    // Flush
	while(port_->read()!=-1);;
    int send = sendMessage(servoId, instruction::READ, 2, readParam);
    // Failed to send
    if (send != 8)
        return -1;
    // Read
    byte result[readLength + 1];
    int rd = recieveMessage(servoId, readLength + 1, result);
    if (rd < 0)
        return rd;

    for (int i = 0; i < readLength; i++)
        outputBuffer[i] = result[i + 1];
    return 0;
}

int STSServoDriver::recieveMessage(byte const& servoId,
                                   byte const& readLength,
                                   byte *outputBuffer)
{
    digitalWrite(dirPin_, LOW);
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