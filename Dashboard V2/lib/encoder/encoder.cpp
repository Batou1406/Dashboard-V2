    #include <encoder.hpp>

/*****************************************************************************************
 *  @brief      Create a encoder object on a given I2C bus
 *  @param      BusI2C the I2C bus connected to the encoder
 *  @param      addr the I2C address of the encoder
 *  @param      buttonPin Pin where the button of the encoder is connected to
 ****************************************************************************************/
Encoder::Encoder(TwoWire* busI2C, uint8_t addr, uint8_t buttonPin, bool debug) {
    _busI2C = busI2C;
    _addr = addr;
    _buttonPin = buttonPin;
    _debug = debug;
    #ifdef ARDUINO_ARCH_SAMD
        _maxBufferSize = 250; // as defined in Wire.h's RingBuffer
    #elif defined(ESP32)
        _maxBufferSize = I2C_BUFFER_LENGTH;
    #else
        _maxBufferSize = 32;
    #endif
}

/*****************************************************************************************
 *  @brief      Start the encoder This should be called when your sketch is starting
 ****************************************************************************************/
void Encoder::init(){
    if(_debug){Serial.println("Looking for the Encoder!");}

    unsigned long timestamp = millis();
    while (millis() < (timestamp + 1000)) {
        if (this->_init()) {
            enable = true;
            break;
        }
        delay(10);

        if(_debug){Serial.println("Looking for the Encoder!");}
    }

    if (enable == false) {
        if(_debug){Serial.println("Timeout: Encoder not found");}
        return;
    }

    if(_debug){Serial.println("Encoder started");}
}


/*****************************************************************************************
 *  @brief      Start the encoder This should be called when your sketch is starting
 *  @return     true if we could connect to the encoder, false otherwise
 ****************************************************************************************/
bool Encoder::_init() {    
    // Set the button mode as input
    //pinMode(_buttonPin, INPUT_PULLUP); // ça fait bugger...

    // Start the I2CBus
    bool found = false;
    for (int tries = 0; tries < 10; tries++){
        Serial.println(tries);
        _busI2C->begin();
        _busI2C->beginTransmission(_addr);
        if(_busI2C->endTransmission() == 0){ // A object is connected to that address
            found = true;
            break;
        }
        delay(10);
    }
    if(!found){
        return false;
    }

    // Perform a software reset : reset all registers from encoder to default values
    return this->write8(0x00, 0x7F, 0xFF);
}

/******************************************************************************************
 *  @brief      Read the current position of the encoder
 *  @param      encoder Which encoder to use, defaults to 0
 *  @return     The encoder position as a 32 bit signed integer.
 ****************************************************************************************/
int32_t Encoder::getEncoderPosition(uint8_t encoder) {
    uint8_t buf[4];
    this->read(ENCODER_BASE, ENCODER_POSITION + encoder, buf, 4);
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return ret;
}

/*****************************************************************************************
 *  @brief      Set the current position of the encoder
 *  @param      encoder Which encoder to use, defaults to 0
 *  @param      pos the position to set the encoder to.
 ****************************************************************************************/
void Encoder::setEncoderPosition(int32_t pos, uint8_t encoder) {
    uint8_t buf[] = {(uint8_t)(pos >> 24), (uint8_t)(pos >> 16), (uint8_t)(pos >> 8), (uint8_t)(pos & 0xFF)};
    this->write(ENCODER_BASE, ENCODER_POSITION + encoder, buf, 4);
}

/*****************************************************************************************
 *  @brief      Read the change in encoder position since it was last read.
 *  @param      encoder Which encoder to use, defaults to 0
 *  @return     The encoder change as a 32 bit signed integer.
 ****************************************************************************************/
int32_t Encoder::getEncoderDelta(uint8_t encoder) {
    uint8_t buf[4];
    this->read(ENCODER_BASE, ENCODER_DELTA + encoder, buf, 4);
    int32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
  return ret;
}

/*****************************************************************************************
 *  @brief      Enable the interrupt to fire when the encoder changes position.
 *  @param      encoder Which encoder to use, defaults to 0
 *  @returns    True on I2C write success
 ****************************************************************************************/
bool Encoder::enableEncoderInterrupt(uint8_t encoder) {
    return this->write8(ENCODER_BASE, ENCODER_INTENSET + encoder, 0x01);
}

/******************************************************************************************
 *  @brief      Disable the interrupt from firing when the encoder changes position.
 *  @param      encoder Which encoder to use, defaults to 0
 *  @returns    True on I2C write success
 ****************************************************************************************/
bool Encoder::disableEncoderInterrupt(uint8_t encoder) {
    return this->write8(ENCODER_BASE, ENCODER_INTENCLR + encoder, 0x01);
}

/******************************************************************************************
 *  @brief      Write 1 byte to the specified seesaw register.
 *  @param      regHigh the module address register 
 *	@param		regLow the function address register 
 *	@param		value the value between 0 and 255 to write
 *  @returns    True on I2C write success
 ****************************************************************************************/
bool Encoder::write8(uint8_t regHigh, uint8_t regLow, uint8_t value) {
    return this->write(regHigh, regLow, &value, 1);
}


/******************************************************************************************
 *  @brief      read 1 byte from the specified seesaw register.
 *  @param      regHigh the module address register 
 *	@param		regLow the function address register 
 *	@param		delay a number of microseconds to delay before reading
 *              out the data. Different delay values may be necessary to ensure the seesaw
 *              chip has time to process the requested data. Defaults to 125.
 *  @return     the value between 0 and 255 read from the passed register
 ****************************************************************************************/
uint8_t Encoder::read8(uint8_t regHigh, uint8_t regLow, uint16_t delay) {
    uint8_t ret;
    this->read(regHigh, regLow, &ret, 1, delay);
    return ret;
}

/****************************************************************************************
 *  @brief      Read a specified number of bytes into a buffer from the seesaw.
 *  @param      regHigh the module address register
 *	@param		regLow the function address register 
 *	@param		buf the buffer to read the bytes into
 *	@param		num the number of bytes to read.
 *	@param		delay an optional delay in between setting the read
 *              register and reading out the data. This is required for some seesaw functions
 *  @returns    True on I2C read success
 ****************************************************************************************/
bool Encoder::read(uint8_t regHigh, uint8_t regLow, uint8_t *buf, uint8_t num, uint16_t delay) {
    uint8_t pos = 0;
    uint8_t prefix[2];
    prefix[0] = (uint8_t)regHigh;
    prefix[1] = (uint8_t)regLow;

  // on arduino we need to read in 32 byte chunks
    while (pos < num) {
        uint8_t read_now = min(32, num - pos);

        // Write prefix
        _busI2C->beginTransmission(_addr);
        if(_busI2C->write(prefix,2) != 2){
            return false;
        }
        if (_busI2C->endTransmission(true) != 0) {
            return false;
        }

        // TODO: tune this
        delayMicroseconds(delay);

        if (!readHelper(buf + pos, read_now)) {
            return false;
        }
        pos += read_now;
    }
    return true;
}

/******************************************************************************************
 *    @brief  Read from I2C into a buffer from the I2C device. Cannot be more than maxBufferSize() bytes.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  stop Whether to send an I2C STOP signal on read
 *    @return True if read was successful, otherwise false.
 *****************************************************************************************/
bool Encoder::readHelper(uint8_t *buffer, size_t len, bool stop) {
    size_t pos = 0;
    while (pos < len) {
        size_t read_len = ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
        bool read_stop = (pos < (len - read_len)) ? false : stop;
        if (!_readHelper(buffer + pos, read_len, read_stop))
            return false;
        pos += read_len;
    }
    return true;
}

bool Encoder::_readHelper(uint8_t *buffer, size_t len, bool stop) {
    #if defined(TinyWireM_h)
        size_t recv = _busI2C->requestFrom((uint8_t)_addr, (uint8_t)len);
    #elif defined(ARDUINO_ARCH_MEGAAVR)
        size_t recv = _busI2C->requestFrom(_addr, len, stop);
    #else
        size_t recv = _busI2C->requestFrom((uint8_t)_addr, (uint8_t)len, (uint8_t)stop);
    #endif

    if (recv != len) {
        // Not enough data available to fulfill our obligation!
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print(F("\tI2CDevice did not receive enough data: "));
            DEBUG_SERIAL.println(recv);
        #endif
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = _busI2C->read();
    }

    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print(F("\tI2CREAD  @ 0x"));
        DEBUG_SERIAL.print(_addr, HEX);
        DEBUG_SERIAL.print(F(" :: "));
        for (uint16_t i = 0; i < len; i++) {
            DEBUG_SERIAL.print(F("0x"));
            DEBUG_SERIAL.print(buffer[i], HEX);
            DEBUG_SERIAL.print(F(", "));
            if (len % 32 == 31) {
                DEBUG_SERIAL.println();
            }
        }
        DEBUG_SERIAL.println();
    #endif

    return true;
}

/*****************************************************************************************
 *  @brief      Write a specified number of bytes to the seesaw from the passed buffer.
 *  @param      regHigh the module address register (ex. SEESAW_GPIO_BASE)
 *  @param	    regLow the function address register (ex. SEESAW_GPIO_BULK_SET)
 *  @param	    buf the buffer the the bytes from
 *  @param	    num the number of bytes to write.
 *  @returns    True on I2C write success
 ****************************************************************************************/
bool Encoder::write(uint8_t regHigh, uint8_t regLow, uint8_t *buf = NULL, uint8_t num = 0) {
    uint8_t prefix[2];
    prefix[0] = (uint8_t)regHigh;
    prefix[1] = (uint8_t)regLow;

    if(!writeHelper(buf, num, true, prefix, 2)) {
        return false;
    }

    return true;
}

/*******************************************************************************
 *    @brief    Write a buffer or two to the I2C device. Cannot be more than maxBufferSize() bytes.
 *    @param    buffer Pointer to buffer of data to write. This is const to ensure the content of this buffer doesn't change.
 *    @param    len Number of bytes from buffer to write
 *    @param    prefix_buffer Pointer to optional array of data to write before buffer. Cannot be more than maxBufferSize()
 *              bytes. This is const to ensure the content of this buffer doesn't change.
 *    @param    prefix_len Number of bytes from prefix buffer to write
 *    @param    stop Whether to send an I2C STOP signal on write
 *    @return   True if write was successful, otherwise false.
 ******************************************************************************/
bool Encoder::writeHelper(const uint8_t *buffer, size_t len, bool stop, const uint8_t *prefix_buffer, size_t prefix_len) {
    if ((len + prefix_len) > maxBufferSize()) {
        // currently not guaranteed to work if more than 32 bytes!
        // we will need to find out if some platforms have larger
        // I2C buffer sizes :/
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.println(F("\tI2CDevice could not write such a large buffer"));
        #endif
        return false;
    }

    _busI2C->beginTransmission(_addr);

    // Write the prefix data (usually an address)
    if ((prefix_len != 0) && (prefix_buffer != nullptr)) {
        if (_busI2C->write(prefix_buffer, prefix_len) != prefix_len) {
            #ifdef DEBUG_SERIAL
                DEBUG_SERIAL.println(F("\tI2CDevice failed to write"));
            #endif
            return false;
        }
    }

    // Write the data itself
    if (_busI2C->write(buffer, len) != len) {
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.println(F("\tI2CDevice failed to write"));
        #endif
        return false;
    }

    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print(F("\tI2CWRITE @ 0x"));
        DEBUG_SERIAL.print(_addr, HEX);
        DEBUG_SERIAL.print(F(" :: "));
        if ((prefix_len != 0) && (prefix_buffer != nullptr)) {
            for (uint16_t i = 0; i < prefix_len; i++) {
                DEBUG_SERIAL.print(F("0x"));
                DEBUG_SERIAL.print(prefix_buffer[i], HEX);
                DEBUG_SERIAL.print(F(", "));
            }
        }
        for (uint16_t i = 0; i < len; i++) {
            DEBUG_SERIAL.print(F("0x"));
            DEBUG_SERIAL.print(buffer[i], HEX);
            DEBUG_SERIAL.print(F(", "));
            if (i % 32 == 31) {
                DEBUG_SERIAL.println();
            }
        }

        if (stop) {
            DEBUG_SERIAL.print("\tSTOP");
        }
    #endif

    if (_busI2C->endTransmission(stop) == 0) {
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.println();
            // DEBUG_SERIAL.println("Sent!");
        #endif
        return true;
    } else {
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.println("\tFailed to send!");
        #endif
        return false;
    }
}

/***********************************************************************
 *  @brief      Read the current status of a GPIO pin
 *  @param      pin the pin number.
 *  @return     the status of the pin. HIGH or LOW (1 or 0).
 ***********************************************************************/
bool Encoder::digitalRead(uint8_t pin) {
  if (pin >= 32)
    return digitalReadBulkB((1ul << (pin - 32))) != 0;
  else
    return digitalReadBulk((1ul << pin)) != 0;
}

/********************************************************************
 *  @brief      read the status of multiple pins on port A.
 *  @param      pins a bitmask of the pins to write. (eg 0b0110 for pins 2 and 3)
 *  @return     the status of the passed pins. If 0b0110 was passed and pin 2 is
 *              high and pin 3 is low, 0b0010 (decimal number 2) will be returned.
 *******************************************************************/
uint32_t Encoder::digitalReadBulk(uint32_t pins) {
  uint8_t buf[4];
  this->read(GPIO_BASE, GPIO_BULK, buf, 4);
  uint32_t ret = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
  return ret & pins;
}

/*************************************************************************
 *  @brief      read the status of multiple pins on port B.
 *  @param      pins a bitmask of the pins to write.
 *  @return     the status of the passed pins. If 0b0110 was passed and pin 2 is
 *              high and pin 3 is low, 0b0010 (decimal number 2) will be returned.
 ************************************************************************/
uint32_t Encoder::digitalReadBulkB(uint32_t pins) {
  uint8_t buf[8];
  this->read(GPIO_BASE, GPIO_BULK, buf, 8);
  uint32_t ret = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
                 ((uint32_t)buf[6] << 8) | (uint32_t)buf[7];
  return ret & pins;
}