#pragma once
#include "BusTypes.h"


    Status i2cInit(I2CConfig& cfg);
    Status i2cWrite(I2CConfig& cfg, const uint8_t* data, size_t len);
    Status i2cRead(I2CConfig& cfg, uint8_t* data, size_t len);
    Status i2cWriteRead(I2CConfig& cfg,
                        const uint8_t* tx, size_t txLen,
                        uint8_t* rx, size_t rxLen);

