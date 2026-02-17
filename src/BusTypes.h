#pragma once
#include <Arduino.h>
#include <Wire.h>      // TwoWire
#include <SPI.h>       // SPIClass, SPI_MODE0


enum class BusType : uint8_t {
    None = 0,
    I2C,
    SPI
};

enum  Status : uint8_t {
    Ok = 0,
    NotInitialized,
    InvalidArg,
    BusError,
    Timeout,
    ShortTransfer,
    NotSupported
};

/** 
 * - Converts status codes to a short readable string
 * - Helpful for printing debug messages in examples
 */
inline const __FlashStringHelper* toString(Status st) {
    switch (st) {
        case Status::Ok:             return F("Ok");
        case Status::NotInitialized: return F("NotInitialized");
        case Status::InvalidArg:     return F("InvalidArg");
        case Status::BusError:       return F("BusError");
        case Status::Timeout:        return F("Timeout");
        case Status::ShortTransfer:  return F("ShortTransfer");
        case Status::NotSupported:   return F("NotSupported");
        default:                     return F("Unknown");
    }
}

struct I2CConfig {
    TwoWire* wire = nullptr;
    uint8_t address = 0x00;
    uint32_t clockHz = 100000;
    int8_t i2cSDA = -1; // Optional: for boards with multiple I2C ports, specify SDA/SCL pins
    int8_t i2cSCL = -1;
};

struct SPIConfig {
    SPIClass* spi = nullptr;
    int csPin = -1;
    uint32_t clockHz = 1000000;
    int8_t spiMOSI = -1; // Optional: for boards with multiple SPI ports, specify MOSI/MISO/SCK pins
    int8_t spiMISO = -1;
    int8_t spiSCK = -1;
    uint8_t mode = SPI_MODE0;
    uint8_t bitOrder = MSBFIRST;
};

