/*  Written by Baptiste Savioz
 *  april 2023
 *  
 *  Library made for the encoder adafruit seesaw
 */

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <arduino.h>
#include <Wire.h>


#define ENCODER_DEFAULT_ADDRESS (0x49) // Default I2C address
#define GPIO_BASE                0x01
#define GPIO_BULK                0x04
#define ENCODER_BASE             0x11

/** encoder module edge definitions */
enum {
  ENCODER_STATUS = 0x00,
  ENCODER_INTENSET = 0x10,
  ENCODER_INTENCLR = 0x20,
  ENCODER_POSITION = 0x30,
  ENCODER_DELTA = 0x40,
};


class Encoder
{
    public:
        Encoder(TwoWire* busI2C, uint8_t addr, uint8_t buttonPin, bool debug); // constructor
        void init();
        int32_t getEncoderPosition(uint8_t encoder = 0);
        int32_t getEncoderDelta(uint8_t encoder = 0);
        bool enableEncoderInterrupt(uint8_t encoder = 0);
        bool disableEncoderInterrupt(uint8_t encoder = 0);
        void setEncoderPosition(int32_t pos, uint8_t encoder = 0);
        bool digitalRead(uint8_t pin);
        bool enable = false; 
 
    private:
        TwoWire* _busI2C;
        uint8_t _addr;
        uint8_t _buttonPin;
        size_t _maxBufferSize;
        bool _debug;
        bool _init();
        bool write8(uint8_t regHigh, uint8_t regLow, uint8_t value);
        uint8_t read8(uint8_t regHigh, uint8_t regLow, uint16_t delay = 250);
        bool read(uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num, uint16_t delay = 250);
        bool readHelper(uint8_t *buffer, size_t len, bool stop = true);
        bool _readHelper(uint8_t *buffer, size_t len, bool stop);
        bool write(uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num);
        bool writeHelper(const uint8_t *buffer, size_t len, bool stop, const uint8_t *prefix_buffer, size_t prefix_len);
        size_t maxBufferSize() { return _maxBufferSize; }
        uint32_t digitalReadBulk(uint32_t pins);
        uint32_t digitalReadBulkB(uint32_t pins);
};

#endif // ENCODER_HPP