/// \file STSServoServoDriver.h
/// \brief A simple driver for the STS-serie TTL servos made by Feetech
///
/// \details This code is meant to work as a minimal example for communicating with
///          the STS servos, in particular the low-cost STS-3215 servo.
///          These servos use a communication protocol identical to
///          the Dynamixel serie, but with a different register mapping
///          (due in part to different functionalities like step mode, multiturn...)
#ifndef STSSERVO_DRIVER_H
#define STSSERVO_DRIVER_H

#include <Arduino.h>

namespace STSRegisters
{
    byte const FIRMWARE_MAJOR           = 0x00;
    byte const FIRMWARE_MINOR           = 0x01;
    byte const SERVO_MAJOR              = 0x03;
    byte const SERVO_MINOR              = 0x04;
    byte const ID                       = 0x05;
    byte const BAUDRATE                 = 0x06;
    byte const RESPONSE_DELAY           = 0x07;
    byte const RESPONSE_STATUS_LEVEL    = 0x08;
    byte const MINIMUM_ANGLE            = 0x09;
    byte const MAXIMUM_ANGLE            = 0x0B;
    byte const MAXIMUM_TEMPERATURE      = 0x0D;
    byte const MAXIMUM_VOLTAGE          = 0x0E;
    byte const MINIMUM_VOLTAGE          = 0x0F;
    byte const MAXIMUM_TORQUE           = 0x10;
    byte const UNLOADING_CONDITION      = 0x13;
    byte const LED_ALARM_CONDITION      = 0x14;
    byte const POS_PROPORTIONAL_GAIN    = 0x15;
    byte const POS_DERIVATIVE_GAIN      = 0x16;
    byte const POS_INTEGRAL_GAIN        = 0x17;
    byte const MINIMUM_STARTUP_FORCE    = 0x18;
    byte const CK_INSENSITIVE_AREA      = 0x1A;
    byte const CCK_INSENSITIVE_AREA     = 0x1B;
    byte const CURRENT_PROTECTION_TH    = 0x1C;
    byte const ANGULAR_RESOLUTION       = 0x1E;
    byte const POSITION_CORRECTION      = 0x1F;
    byte const OPERATION_MODE           = 0x21;
    byte const TORQUE_PROTECTION_TH     = 0x22;
    byte const TORQUE_PROTECTION_TIME   = 0x23;
    byte const OVERLOAD_TORQUE          = 0x24;
    byte const SPEED_PROPORTIONAL_GAIN  = 0x25;
    byte const OVERCURRENT_TIME         = 0x26;
    byte const SPEED_INTEGRAL_GAIN      = 0x27;
    byte const TORQUE_SWITCH            = 0x28;
    byte const TARGET_ACCELERATION      = 0x29;
    byte const TARGET_POSITION          = 0x2A;
    byte const RUNNING_TIME             = 0x2C;
    byte const RUNNING_SPEED            = 0x2E;
    byte const TORQUE_LIMIT             = 0x30;
    byte const WRITE_LOCK               = 0x37;
    byte const CURRENT_POSITION         = 0x38;
    byte const CURRENT_SPEED            = 0x3A;
    byte const CURRENT_DRIVE_VOLTAGE    = 0x3C;
    byte const CURRENT_VOLTAGE          = 0x3E;
    byte const CURRENT_TEMPERATURE      = 0x3F;
    byte const ASYNCHRONOUS_WRITE_ST    = 0x40;
    byte const STATUS                   = 0x41;
    byte const MOVING_STATUS            = 0x42;
    byte const CURRENT_CURRENT          = 0x45;
};

enum STSMode{
    POSITION = 0,
    VELOCITY = 1,
    STEP = 3
};

enum ServoType
{
    UNKNOWN = 0,
    STS = 1,
    SCS = 2
};

/// \brief Driver for STS servos, using UART
class STSServoDriver
{
public:
    /// \brief Constructor.
    STSServoDriver();

    /// \brief Initialize the servo driver.
    ///
    /// \param dirPin Pin used for setting communication direction
    /// \param serialPort Serial port, default is Serial
    /// \param baudRate Baud rate, default 1Mbps
    /// \returns  True on success (at least one servo responds to ping)
    bool init(byte const &dirPin, HardwareSerial *serialPort = nullptr, long const &baudRate = 1000000);

    /// \brief Initialize the servo driver without direction pin.
    /// when you use an interface board including FE-URT-1, direction pin is not required.
    /// The board controls the direction of signals to FeeTech serial servos.
    /// In this method, when direction pin number is 255, the pin is inactive.  
    /// \param serialPort Serial port, default is Serial
    /// \param baudRate Baud rate, default 1Mbps
    /// \returns  True on success (at least one servo responds to ping)
    bool init(HardwareSerial *serialPort = nullptr, long const &baudRate = 1000000);

    /// \brief Ping servo
    /// \param[in] servoId ID of the servo
    /// \return True if servo responded to ping
    bool ping(byte const &servoId);

    /// \brief Change the ID of a servo.
    /// \note If the desired ID is already taken, this function does nothing and returns false.
    /// \param[in] oldServoId old servo ID
    /// \param[in] newServoId new servo ID
    /// \return True if servo could successfully change ID
    bool setId(byte const &oldServoId, byte const &newServoId);

    /// \brief Change the position offset of a servo.
    /// \param[in] servoId servo ID
    /// \param[in] positionOffset new position offset
    /// \return True if servo could successfully change position offset
    bool setPositionOffset(byte const &servoId, int const &positionOffset);

    /// \brief Get current servo position.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \return Position, in counts. 0 on failure.
    int getCurrentPosition(byte const &servoId);

    /// \brief Get current servo speed.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \return Speed, in counts/s. 0 on failure.
    int getCurrentSpeed(byte const &servoId);

    /// \brief Get current servo temperature.
    /// \param[in] servoId ID of the servo
    /// \return Temperature, in degC. 0 on failure.
    int getCurrentTemperature(byte const &servoId);

    /// \brief Get current servo current.
    /// \param[in] servoId ID of the servo
    /// \return Current, in A.
    float getCurrentCurrent(byte const &servoId);

    /// \brief Check if the servo is moving
    /// \param[in] servoId ID of the servo
    /// \return True if moving, false otherwise.
    bool isMoving(byte const &servoId);

    /// \brief Set target servo position.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \param[in] position Target position, in counts.
    /// \param[in] speed speed of the servo.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True on success, false otherwise.
    bool setTargetPosition(byte const &servoId, int const &position, int const &speed = 4095, bool const &asynchronous = false);

    /// \brief Set target servo velocity.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \param[in] velocity Target velocity, in counts/s.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True on success, false otherwise.
    bool setTargetVelocity(byte const &servoId, int const &velocity, bool const &asynchronous = false);

    /// \brief Change the target acceleration of a servo.
    /// \param[in] servoId servo ID
    /// \param[in] acceleration target acceleration
    /// \return True if servo could successfully set target acceleration
    bool setTargetAcceleration(byte const &servoId, byte const &acceleration, bool const &asynchronous = false);


    /// \brief Set servo working mode: position, velocity or step.
    /// \param[in] servoId ID of the servo
    /// \param[in] mode Desired mode
    bool setMode(unsigned char const& servoId, STSMode const& mode);

    /// \brief Trigger the action previously stored by an asynchronous write on all servos.
    /// \return True on success
    bool trigerAction();

    /// \brief Write to a single byte register.
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId Register id.
    /// \param[in] value Register value.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True if write was successful
    bool writeRegister(byte const &servoId,
                       byte const &registerId,
                       byte const &value,
                       bool const &asynchronous = false);

    /// \brief Write a two-bytes register.
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId Register id (LSB).
    /// \param[in] value Register value.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True if write was successful
    bool writeTwoBytesRegister(byte const &servoId,
                               byte const &registerId,
                               int16_t const &value,
                               bool const &asynchronous = false);

    /// \brief Read a single register
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId Register id.
    /// \return Register value, 0 on failure.
    byte readRegister(byte const &servoId, byte const &registerId);

    /// \brief Read two bytes, interpret result as <LSB> <MSB>
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId LSB register id.
    /// \return Register value, 0 on failure.
    int16_t readTwoBytesRegister(byte const &servoId, byte const &registerId);

    /// @brief Sets the target positions for multiple servos simultaneously.
    /// @param[in] NumberOfServos Number of servo.
    /// @param[in] servoIds Array of servo IDs to control.
    /// @param[in] positions Array of target positions (corresponds to servoIds).
    /// @param[in] speeds Array of target speeds (corresponds to servoIds).
    void setTargetPositions(byte const &numberOfServos,
                            const byte servoIds[],
                            const int positions[],
                            const int speeds[]);

private:
    /// \brief Send a message to the servos.
    /// \param[in] servoId ID of the servo
    /// \param[in] commandID Command id
    /// \param[in] paramLength length of the parameters
    /// \param[in] parameters parameters
    /// \return Result of write.
    int sendMessage(byte const &servoId,
                    byte const &commandID,
                    byte const &paramLength,
                    byte *parameters);

    /// \brief Recieve a message from a given servo.
    /// \param[in] servoId ID of the servo
    /// \param[in] readLength Message length
    /// \param[in] paramLength length of the parameters
    /// \param[in] outputBuffer Buffer where the data is placed.
    /// \return 0 on success
    ///         -1 if read failed due to timeout
    ///         -2 if invalid message (no 0XFF, wrong servo id)
    ///         -3 if invalid checksum
    int receiveMessage(byte const &servoId,
                       byte const &readLength,
                       byte *outputBuffer);

    /// \brief Write to a sequence of consecutive registers
    /// \param[in] servoId ID of the servo
    /// \param[in] startRegister First register
    /// \param[in] writeLength Number of registers to write
    /// \param[in] parameters Value of the registers
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True if write was successful
    bool writeRegisters(byte const &servoId,
                        byte const &startRegister,
                        byte const &writeLength,
                        byte const *parameters,
                        bool const &asynchronous = false);

    /// \brief Read a sequence of consecutive registers.
    /// \param[in] servoId ID of the servo
    /// \param[in] startRegister First register
    /// \param[in] readLength Number of registers to write
    /// \param[out] outputBuffer Buffer where to read the data (must have been allocated by the user)
    /// \return 0 on success, -1 if write failed, -2 if read failed, -3 if checksum verification failed
    int readRegisters(byte const &servoId,
                      byte const &startRegister,
                      byte const &readLength,
                      byte *outputBuffer);

    /// @brief Send two bytes and update checksum
    /// @param[in] convertedValue Converted int value
    /// @param[out] checksum Update the checksum
    void sendAndUpdateChecksum(byte convertedValue[], byte &checksum);

    /// @brief Convert int to pair of bytes
    /// @param[in] value
    /// @param[out] result
    void convertIntToBytes(byte const& servoId, int const &value, byte result[2]);

    /// \brief Determine servo type (STS or SCS, they don't use exactly the same protocol)
    void determineServoType(byte const& servoId);

    HardwareSerial *port_;
    byte dirPin_; ///< Direction pin number.

    ServoType servoType_[256]; // Map of servo types - STS/SCS servos have slightly different protocol.
};
#endif
