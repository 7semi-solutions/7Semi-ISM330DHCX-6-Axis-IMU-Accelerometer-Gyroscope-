#pragma once
#include "BusTypes.h"
#include <Wire.h>
#include <SPI.h>

class BusDriver {
public:
    BusDriver();

    /** 
     * - Initializes I2C bus and stores device address
     * - clockHz can be 100000 or 400000 for most sensors
     */
    Status beginI2C(TwoWire& wire, uint8_t address, uint32_t clockHz = 100000);

    /**
     * - Initializes SPI bus and configures CS pin
     * - Uses SPI transaction per call, safe for multiple SPI devices
     */
    Status beginSPI(SPIClass& bus_spi, int csPin,
                    uint32_t clockHz = 1000000,
                    uint8_t mode = SPI_MODE0,
                    uint8_t bitOrder = MSBFIRST);

    /**
     * - Initializes UART bus with baud rate and read timeout
     * - timeoutMs controls read() and writeRead() maximum waiting time
     */
    Status beginUART(HardwareSerial& serial,
                     uint32_t baud = 115200,
                     uint32_t timeoutMs = 20);

    /** 
     * - Resets the driver to BusType::None
     * - Does not de-initialize Arduino core peripherals
     */
    void end();

    Status write(const uint8_t* data, size_t len);
    Status read(uint8_t* data, size_t len);
    Status writeRead(const uint8_t* tx, size_t txLen,
                     uint8_t* rx, size_t rxLen);

    /** 
     * - Convenience helpers for register-based devices
     * - For SPI sensors, your sensor library may need to set read/write bits in reg value
     */
    Status readReg(uint8_t reg, uint8_t* data, size_t len);
    Status writeReg(uint8_t reg, const uint8_t* data, size_t len);

    /** 
     * - Updates I2C device address after beginI2C()
     * - Useful when one driver supports multiple I2C addresses
     */
    Status setI2CAddress(uint8_t address);

    bool isReady() const { return driver != BusType::None; }
    BusType type() const { return driver; }
enum ISM330_Rounding : uint8_t {
    ROUNDING_NONE = 0,
    ROUNDING_XL   = 1,
    ROUNDING_G    = 2,
    ROUNDING_BOTH = 3
};

enum ISM330_SelfTest : uint8_t {
    ST_NORMAL   = 0,
    ST_POSITIVE = 1,
    ST_NEGATIVE = 2,
};

private:
    BusType driver;

    I2CConfig  i2c;
    SPIConfig  spi;
};


