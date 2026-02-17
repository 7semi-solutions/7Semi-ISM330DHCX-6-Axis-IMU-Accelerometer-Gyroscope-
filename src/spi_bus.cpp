#include "spi_bus.h"

static inline void csLow(const SPIConfig &c) { digitalWrite(c.csPin, LOW); }
static inline void csHigh(const SPIConfig &c) { digitalWrite(c.csPin, HIGH); }

Status spiInit(SPIConfig &cfg)
{
    if (!cfg.spi)
    {
        return Status::InvalidArg;
    }
    pinMode(cfg.csPin, OUTPUT);
    csHigh(cfg);
#if defined(ARDUINO_ARCH_ESP32)
    if (cfg.spiMOSI >= 0 && cfg.spiMISO >= 0 && cfg.spiSCK >= 0)
    {
        cfg.spi->begin(cfg.spiSCK, cfg.spiMISO, cfg.spiMOSI);
    }
    else
    {
        cfg.spi->begin();
    }
#else
    // AVR, ESP8266, SAMD, RP2040, STM32
    cfg.spi->begin();
#endif
    return Status::Ok;
}

Status spiWrite(SPIConfig &cfg, const uint8_t *data, size_t len)
{
    cfg.spi->beginTransaction(SPISettings(cfg.clockHz, cfg.bitOrder, cfg.mode));
    csLow(cfg);
    for (size_t i = 0; i < len; i++)
        cfg.spi->transfer(data[i]);
    csHigh(cfg);
    cfg.spi->endTransaction();
    return Status::Ok;
}

Status spiRead(SPIConfig &cfg, uint8_t *data, size_t len)
{
    cfg.spi->beginTransaction(SPISettings(cfg.clockHz, cfg.bitOrder, cfg.mode));
    csLow(cfg);
    for (size_t i = 0; i < len; i++)
        data[i] = cfg.spi->transfer(0x00);
    csHigh(cfg);
    cfg.spi->endTransaction();
    return Status::Ok;
}

Status spiWriteRead(SPIConfig &cfg,
                    const uint8_t *tx, size_t txLen,
                    uint8_t *rx, size_t rxLen)
{
    cfg.spi->beginTransaction(SPISettings(cfg.clockHz, cfg.bitOrder, cfg.mode));
    csLow(cfg);
    for (size_t i = 0; i < txLen; i++)
        cfg.spi->transfer(tx[i]);
    for (size_t i = 0; i < rxLen; i++)
        rx[i] = cfg.spi->transfer(0x00);
    csHigh(cfg);
    cfg.spi->endTransaction();
    return Status::Ok;
}
