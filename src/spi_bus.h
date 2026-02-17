#pragma once
#include "BusTypes.h"


    Status spiInit(SPIConfig& cfg);
    Status spiWrite(SPIConfig& cfg, const uint8_t* data, size_t len);
    Status spiRead(SPIConfig& cfg, uint8_t* data, size_t len);
    Status spiWriteRead(SPIConfig& cfg,
                        const uint8_t* tx, size_t txLen,
                        uint8_t* rx, size_t rxLen);

