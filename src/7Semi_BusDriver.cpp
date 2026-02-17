#include "7Semi_BusDriver.h"
#include "i2c_bus.h"
#include "spi_bus.h"

BusDriver::BusDriver() : driver(BusType::None) {};
Status BusDriver::beginI2C(TwoWire& wire, uint8_t address, uint32_t clockHz) {
    /** 
     * - Initialize I2C bus configuration
     * - address is the sensor/device I2C address (example: 0x68)
     * - clockHz can be 100000 (standard) or 400000 (fast mode)
     * - If clockHz is 0, it will use 100000 by default
     */
    i2c.wire = &wire;
    i2c.address = address;
    i2c.clockHz = clockHz ? clockHz : 100000;

    /** 
     * - Call the I2C bus init handler
     * - On success, this BusDriver becomes an I2C driver
     */
    Status st = i2cInit(i2c);
    if (st == Status::Ok) driver = BusType::I2C;
    return st;
}

Status BusDriver::beginSPI(SPIClass& bus_spi, int csPin, uint32_t clockHz, uint8_t mode, uint8_t bitOrder) {
    /** 
     * - Initialize SPI bus configuration
     * - csPin is the chip select pin connected to the sensor
     * - clockHz is SPI clock speed (example: 1000000 = 1MHz)
     * - mode is SPI_MODE0/1/2/3 depending on the sensor datasheet
     * - bitOrder is usually MSBFIRST for most sensors
     * - If clockHz is 0, it will use 1000000 by default
     */
    spi.spi = &bus_spi;
    spi.csPin = csPin;
    spi.clockHz = clockHz ? clockHz : 1000000;
    spi.mode = mode;
    spi.bitOrder = bitOrder;

    /** 
     * - Call the SPI bus init handler
     * - On success, this BusDriver becomes an SPI driver
     */
    Status st = spiInit(spi);
    if (st == Status::Ok) driver = BusType::SPI;
    return st;
}


void BusDriver::end() {
    /** 
     * - Reset active driver type to None
     * - Does not stop Wire/SPI/Serial objects (Arduino core owns them)
     * - Safe to call multiple times
     */
    driver = BusType::None;
}

Status BusDriver::write(const uint8_t* data, size_t len) {
    /** 
     * - Write raw bytes on the active bus
     * - Returns NotInitialized if begin*() was not called
     * - Returns InvalidArg if data is null or len is 0
     */
    if (driver == BusType::None) return Status::NotInitialized;
    if (!data || len == 0) return Status::InvalidArg;

    /** 
     * - Routes the write call to the correct bus implementation
     * - I2C writes to device address
     * - SPI writes under one CS active window
     * - UART writes to serial port
     */
    switch (driver) {
        case BusType::I2C:  return i2cWrite(i2c, data, len);
        case BusType::SPI:  return spiWrite(spi, data, len);
        default:            return Status::NotSupported;
    }
}

Status BusDriver::read(uint8_t* data, size_t len) {
    /** 
     * - Read raw bytes from the active bus
     * - Returns NotInitialized if begin*() was not called
     * - Returns InvalidArg if data is null or len is 0
     */
    if (driver == BusType::None) return Status::NotInitialized;
    if (!data || len == 0) return Status::InvalidArg;

    /** 
     * - Routes the read call to the correct bus implementation
     * - I2C reads from device address
     * - SPI reads by sending dummy bytes (0x00)
     * - UART reads until len bytes arrive or timeout happens
     */
    switch (driver) {
        case BusType::I2C:  return i2cRead(i2c, data, len);
        case BusType::SPI:  return spiRead(spi, data, len);
        default:            return Status::NotSupported;
    }
}

Status BusDriver::writeRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) {
    /** 
     * - Common sensor operation:
     *   - write some bytes (register address / command)
     *   - then read response bytes
     * - tx can be null if txLen is 0
     * - rx can be null if rxLen is 0
     */
    if (driver == BusType::None) return Status::NotInitialized;
    if (txLen && !tx) return Status::InvalidArg;
    if (rxLen && !rx) return Status::InvalidArg;

    /** 
     * - Routes writeRead to the correct bus implementation
     * - I2C does write then requestFrom
     * - SPI does one CS window transfer
     * - UART does write then read with timeout
     */
    switch (driver) {
        case BusType::I2C:  return i2cWriteRead(i2c, tx, txLen, rx, rxLen);
        case BusType::SPI:  return spiWriteRead(spi, tx, txLen, rx, rxLen);
        default:            return Status::NotSupported;
    }
}

Status BusDriver::readReg(uint8_t reg, uint8_t* data, size_t len) {
    /** 
     * - Helper for register-based devices
     * - Sends 1 byte register address, then reads len bytes
     * - Works directly for I2C sensors
     * - For SPI sensors, your device may need reg read-bit handling in the sensor library
     */
    return writeRead(&reg, 1, data, len);
}

Status BusDriver::writeReg(uint8_t reg, const uint8_t* data, size_t len) {
    /** 
     * - Helper for register-based devices
     * - Writes register address, then writes payload bytes
     * - Returns InvalidArg if data is null or len is 0
     */
    if (!data || len == 0) return Status::InvalidArg;

    /** 
     * - First write the register address
     * - Then write the register payload
     */
    Status st = write(&reg, 1);
    if (st != Status::Ok) return st;
    return write(data, len);
}

Status BusDriver::setI2CAddress(uint8_t address) {
    /** 
     * - Change I2C device address after beginI2C()
     * - Useful when one driver supports multiple address options
     * - Returns NotSupported if current bus is not I2C
     */
    if (driver != BusType::I2C) return Status::NotSupported;
    i2c.address = address;
    return Status::Ok;
}
