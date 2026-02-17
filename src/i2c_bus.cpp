#include "i2c_bus.h"
Status i2cInit(I2CConfig &cfg)
{
    if (!cfg.wire)
    {
        return Status::InvalidArg;
    }
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    if (cfg.i2cSDA >= 0 && cfg.i2cSCL >= 0)
        cfg.wire->setPins(cfg.i2cSDA, cfg.i2cSCL);
    else
        cfg.wire->setPins(SDA, SCL);
#endif
    cfg.wire->begin();
    cfg.wire->setClock(cfg.clockHz);
    cfg.wire->beginTransmission(cfg.address);
    return (cfg.wire->endTransmission(true) == 0) ? Status::Ok : Status::BusError;
}

Status i2cWrite(I2CConfig &cfg, const uint8_t *data, size_t len)
{
    cfg.wire->beginTransmission(cfg.address);
    size_t n = cfg.wire->write(data, len);
    if (n != len)
        return Status::ShortTransfer;
    return (cfg.wire->endTransmission(true) == 0) ? Status::Ok : Status::BusError;
}

Status i2cRead(I2CConfig &cfg, uint8_t *data, size_t len)
{
    uint8_t n = cfg.wire->requestFrom(cfg.address, (uint8_t)len);
    if (n != len)
        return Status::ShortTransfer;
    for (size_t i = 0; i < len; i++)
        data[i] = cfg.wire->read();
    return Status::Ok;
}

Status i2cWriteRead(I2CConfig &cfg,
                    const uint8_t *tx, size_t txLen,
                    uint8_t *rx, size_t rxLen)
{
    if (txLen)
    {
        Status st = i2cWrite(cfg, tx, txLen);
        if (st != Status::Ok)
            return st;
    }
    if (rxLen)
        return i2cRead(cfg, rx, rxLen);
    return Status::Ok;
}