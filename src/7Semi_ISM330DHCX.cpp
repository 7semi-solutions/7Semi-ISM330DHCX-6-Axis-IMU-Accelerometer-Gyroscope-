/**
 * 7Semi ISM330DHCX Driver (Implementation)
 * - Implements the ISM330DHCX_7Semi class declared in 7Semi_ISM330DHCX.h.
 * - Provides:
 *   - WHO_AM_I identity check and default configuration
 *   - Accel/Gyro/Temp conversion into engineering units
 *   - FIFO read helpers designed for tagged FIFO frames (7 bytes each)
 *   - Register read/write helpers built on the 7Semi BusDriver
 *
 * FIFO IMPORTANT:
 * - FIFO output is read from FIFO_DATA_OUT_TAG (0x78) as 7-byte frames:
 * - TAG + X/Y/Z (int16 little-endian)
 */

#include "7Semi_ISM330DHCX.h"

/**
 * ISM330DHCX_7Semi
 * - Lightweight driver wrapper around a shared Bus abstraction.
 * - Supports I2C and SPI transports through beginI2C()/beginSPI().
 * - Provides basic init + accel/gyro/temp reads + FIFO/interrupt helpers.
 */
ISM330DHCX_7Semi::ISM330DHCX_7Semi() {}

/* ========================= BUS INIT ========================= */

/**
 * beginI2C
 * - Initializes the underlying bus in I2C mode.
 * - addr is the 7-bit device address (already shifted as typical Arduino style expects).
 * - clockHz sets I2C clock; use something stable like 100k/400k unless your wiring is solid.
 */
bool ISM330DHCX_7Semi::beginI2C(TwoWire &wire, uint8_t addr, uint32_t clockHz)
{
    return bus.beginI2C(wire, addr, clockHz) == Ok;
}

/**
 * beginSPI
 * - Initializes the underlying bus in SPI mode.
 * - csPin is controlled by the bus layer.
 * - clockHz is SPI SCLK; keep conservative if wires are long/noisy.
 * - Uses SPI_MODE3 + MSBFIRST (matches ISM330DHCX typical SPI requirements).
 */
bool ISM330DHCX_7Semi::beginSPI(SPIClass &spi, int csPin, uint32_t clockHz)
{
    return bus.beginSPI(spi, csPin, clockHz, SPI_MODE3, MSBFIRST) == Ok;
}

/* ========================= LOW LEVEL ========================= */

/**
 * writeReg
 * - Writes a single register with a single byte payload.
 * - For I2C: sends {reg, val}.
 * - For SPI: bus layer is expected to handle CS + transaction framing.
 */
bool ISM330DHCX_7Semi::writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return bus.write(buf, 2) == Ok;
}

/**
 * readReg
 * - Reads a single byte from a register.
 * - For SPI reads, sets the READ bit in the address.
 * - For I2C reads, uses normal register addressing.
 */
bool ISM330DHCX_7Semi::readReg(uint8_t reg, uint8_t &data)
{
    uint8_t addr = reg;

    if (bus.type() == BusType::SPI)
    {
        /**
         * SPI register read
         * - Bit7 = 1 enables read.
         */
        addr |= 0x80;
    }

    return bus.writeRead(&addr, 1, &data, 1) == Ok;
}

/**
 * readRegs
 * - Burst reads len bytes starting at reg into buf.
 * - For SPI burst reads, sets READ + AUTO-INCREMENT bits.
 * - For I2C burst reads, bus layer performs repeated read from auto-incremented registers.
 */
bool ISM330DHCX_7Semi::readRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t addr = reg;

    if (bus.type() == BusType::SPI)
    {
        /**
         * SPI burst read
         * - Bit7 = 1 enables read
         * - Bit6 = 1 enables address auto-increment
         */
        addr |= 0xC0;
    }

    return bus.writeRead(&addr, 1, buf, len) == Ok;
}
bool ISM330DHCX_7Semi::writeBit(uint8_t reg, uint8_t bit_position, bool value)
{
    if (bit_position > 7)
        return false;

    uint8_t v;
    if (!readReg(reg, v))
        return false;

    if (value)
        v |= (1U << bit_position); // Set the bit
    else
        v &= ~(1U << bit_position); // Clear the bit
    return writeReg(reg, v);
}

bool ISM330DHCX_7Semi::getBit(uint8_t reg, uint8_t bit_position, bool &bit_value)
{
    if (bit_position > 7)
        return false;

    uint8_t v;
    if (!readReg(reg, v))
        return false;

    bit_value = (v & (1U << bit_position)) != 0U;

    return true;
}
static inline int16_t bytesToInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}
/* ========================= DEVICE INIT ========================= */

/**
 * begin
 * - Verifies WHO_AM_I matches expected product ID.
 * - Issues software reset and waits for the device to reboot.
 * - Enables:
 *   - BDU (Block Data Update) to prevent partially-updated multi-byte reads
 *   - IF_INC (auto-increment) for burst reads
 * - Applies default accel + gyro config (ODR 833 Hz).
 */
bool ISM330DHCX_7Semi::begin(TwoWire &wire, uint8_t addr, uint32_t clockHz)
{
    uint8_t id;

    if (!beginI2C(wire, addr, clockHz))
        return false;

    /**
     * Device identity check
     * - If this fails, wiring/bus config/address is the first suspect.
     */
    if (!getProductID(id))
        return false;
    if (id != ISM330DHCX_ID)
        return false;

    /**
     * Software reset
     * - Restores registers to default.
     * - Delay is required; device needs time to come back.
     */
    if (!softwareReset())
        return false;
    delay(5);

    /**
     * Safety + convenience configuration
     * - BDU ensures OUTx reads are coherent across L/H bytes.
     * - IF_INC enables multi-byte reads without rewriting address.
     */
    if (!enableBDU(true))
        return false;
    if (!setAutoIncrement(true))
        return false;

    /**
     * Default accelerometer setup
     * - ODR = 833 Hz
     * - FS  = ±2 g
     * - lpf2 enables internal low-pass option (bit mapping is handled in setAccelConfig).
     */
    if (!setAccelConfig(XL_ODR_104Hz, XL_FS_2G, true))
        return false;

    /**
     * Default gyroscope setup
     * - ODR = 833 Hz
     * - FS  = 250 dps
     * - lpf1 enables internal low-pass option (bit mapping is handled in setGyroConfig).
     */
    if (!setGyroConfig(G_ODR_104Hz, G_FS_250, true))
        return false;

    return true;
}

/**
 * getProductID
 * - Reads WHO_AM_I register.
 * - Caller compares against ISM330DHCX_ID.
 */
bool ISM330DHCX_7Semi::getProductID(uint8_t &id)
{
    return readReg(WHO_AM_I, id);
}

/* ========================= SENSOR READ ========================= */

/**
 * readAccel
 * - Reads raw accelerometer XYZ from OUTX_L_A..OUTZ_H_A.
 * - Converts to g using accelGPerLSB() based on xl_fs.
 * - Optional smoothing:
 *   - smoothUpdate() maintains an internal 1D filter state per axis.
 *   - Intended to reduce jitter for UI/slow control loops.
 *
 * Note:
 * - raw LSB order is little-endian (L then H).
 */
bool ISM330DHCX_7Semi::readAccel(float &x, float &y, float &z)
{
    uint8_t raw[6];

    if (!readRegs(OUTX_L_A, raw, sizeof(raw)))
        return false;

    int16_t ax = bytesToInt16(raw[0], raw[1]);
    int16_t ay = bytesToInt16(raw[2], raw[3]);
    int16_t az = bytesToInt16(raw[4], raw[5]);

    /**
     * Optional smoothing path
     * - Keeps filtered estimate in x/y/z (before scaling).
     * - Useful when you want stable readings more than instantaneous response.
     */
    if (smoothEnable)
    {
        x = smoothUpdate(sax, ax);
        y = smoothUpdate(say, ay);
        z = smoothUpdate(saz, az);
    }

    /**
     * Scale to g
     * - Sensitivity depends on full-scale selection.
     * - accelGPerLSB() returns g per LSB (already in g units).
     */
    x = ax * accel_sensitivity;
    y = ay * accel_sensitivity;
    z = az * accel_sensitivity;

    return true;
}

/**
 * readAccelMS2
 * - Convenience wrapper over readAccel().
 * - Converts g to m/s^2 using standard gravity.
 */
bool ISM330DHCX_7Semi::readAccelMS2(float &x, float &y, float &z)
{
    if (!readAccel(x, y, z))
        return false;

    /**
     * Unit conversion
     * - 1 g = 9.80665 m/s^2
     */
    x *= 9.80665f;
    y *= 9.80665f;
    z *= 9.80665f;
    return true;
}

/**
 * readGyro
 * - Reads raw gyroscope XYZ from OUTX_L_G..OUTZ_H_G.
 * - Converts to deg/s using gyroDpsPerLSB() based on g_fs.
 * - Optional smoothing (same approach as accel).
 */
bool ISM330DHCX_7Semi::readGyro(float &x, float &y, float &z)
{
    uint8_t raw[6];

    if (!readRegs(OUTX_L_G, raw, sizeof(raw)))
        return false;

    int16_t gx = bytesToInt16(raw[0], raw[1]);
    int16_t gy = bytesToInt16(raw[2], raw[3]);
    int16_t gz = bytesToInt16(raw[4], raw[5]);

    /**
     * Optional smoothing path
     * - Filters raw integer readings before scaling.
     */
    if (smoothEnable)
    {
        x = smoothUpdate(sgx, gx);
        y = smoothUpdate(sgy, gy);
        z = smoothUpdate(sgz, gz);
    }

    /**
     * Scale to deg/s
     * - Sensitivity depends on full-scale selection.
     */
    x = x * gyro_sensitivity;
    y = y * gyro_sensitivity;
    z = z * gyro_sensitivity;

    return true;
}

/**
 * readGyroRads
 * - Convenience wrapper over readGyro().
 * - Converts deg/s to rad/s.
 */
bool ISM330DHCX_7Semi::readGyroRads(float &x, float &y, float &z)
{
    if (!readGyro(x, y, z))
        return false;

    /**
     * Unit conversion
     * - deg/s to rad/s multiplier = pi/180
     */
    // const float DEG_TO_RAD = 0.017453292519943295f;
    x *= DEG_TO_RAD;
    y *= DEG_TO_RAD;
    z *= DEG_TO_RAD;
    return true;
}

/**
 * readTemp
 * - Reads raw temperature output and converts to °C.
 * - Formula matches common ST IMU convention:
 *   - T(°C) = 25 + (raw / 256)
 */
bool ISM330DHCX_7Semi::readTemp(float &t)
{
    uint8_t raw[2];

    if (!readRegs(OUT_TEMP_L, raw, sizeof(raw)))
        return false;

    int16_t tmp = bytesToInt16(raw[0], raw[1]);
    t = 25.0f + ((float)tmp / 256.0f);

    return true;
}

/* ========================= FIFO ========================= */

/**
 * fifoEnable
 * - Enables FIFO with a selected mode (lower 3 bits).
 * - Also configures FIFO_CTRL3/2 with fixed values used by this library.
 *
 * Note:
 * - The exact meaning of FIFO_CTRLx values depends on your header definitions.
 * - Keep these writes consistent with the datasheet FIFO setup you want (gyro/accel batching, decimation, etc.).
 */
bool ISM330DHCX_7Semi::fifoEnable(uint8_t mode)
{
    uint8_t v = (mode & 0x07) | 0x10; /**
                                       * - mode selects FIFO mode
                                       * - 0x10 enables FIFO (library convention used here)
                                       */
    if (!writeReg(FIFO_CTRL4, v))
        return false;
    if (!writeReg(FIFO_CTRL3, 0x77))
        return false;
    if (!writeReg(FIFO_CTRL2, 0x00))
        return false;
    return true;
}

/**
 * fifoDisable
 * - Disables FIFO by clearing FIFO_CTRL4.
 */
bool ISM330DHCX_7Semi::fifoDisable()
{
    return writeReg(FIFO_CTRL4, 0x00);
}

/**
 * fifoSetWatermark
 * - Sets FIFO watermark threshold.
 * - WTM is 9 bits (low 8 bits in FIFO_CTRL1, high bit in FIFO_CTRL2[0]).
 */
bool ISM330DHCX_7Semi::fifoSetWatermark(uint16_t wtm)
{
    uint8_t low = (uint8_t)(wtm & 0xFF);
    uint8_t high = (uint8_t)((wtm >> 8) & 0x01);

    if (!writeReg(FIFO_CTRL1, low))
        return false;

    uint8_t v;
    if (!readReg(FIFO_CTRL2, v))
        return false;
    v = (uint8_t)((v & ~0x01) | high);
    return writeReg(FIFO_CTRL2, v);
}

bool ISM330DHCX_7Semi::fifoRead(uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0)
        return false;

    uint16_t availableBytes = 0;
    if (!fifoGetLevel(availableBytes))
        return false;

    /**
     * fifoRead
     * - Reads FIFO content in TAG mode (7-byte frames: 1 TAG + 6 DATA).
     * - FIFO data is exposed through the fixed window 0x78..0x7E:
     *   - FIFO_DATA_OUT_TAG (0x78)
     *   - FIFO_DATA_OUT_*   (0x79..0x7E)
     * - IMPORTANT:
     *   - You cannot read *multiple* FIFO frames with one long auto-increment burst,
     *     because register address auto-increment would continue past 0x7E.
     *   - Always read FIFO in 7-byte chunks, restarting from FIFO_DATA_OUT_TAG
     *     for each frame.
     */
    uint16_t n = (uint16_t)min((uint32_t)len, (uint32_t)availableBytes);
    n = (uint16_t)(n - (n % FIFO_TAG_FRAME_BYTES));
    if (n == 0)
        return false;

    uint16_t frames = (uint16_t)(n / FIFO_TAG_FRAME_BYTES);

    for (uint16_t i = 0; i < frames; i++)
    {
        uint8_t *dst = &buf[i * FIFO_TAG_FRAME_BYTES];

        /**
         * - Read exactly one FIFO frame (TAG + 6 data bytes)
         * - Restart at FIFO_DATA_OUT_TAG each time so the device returns the next
         *   FIFO word correctly regardless of IF_INC setting.
         */
        if (!readRegs(FIFO_DATA_OUT_TAG, dst, FIFO_TAG_FRAME_BYTES))
            return false;
    }

    return true;
}

/* ============================================================
 * 2) CPP: implement functions
 * ============================================================ */

/** Put these in 7Semi_ISM330DHCX.cpp */

bool ISM330DHCX_7Semi::fifoGetLevel(uint16_t &level)
{
    uint8_t status[2] = {0};

    if (!readRegs(FIFO_STATUS1, status, 2))
        return false;

    /**
     * fifoGetLevel
     * - FIFO_STATUS1 (LSB) + FIFO_STATUS2[1:0] (MSB) form DIFF_FIFO_[9:0].
     * - Per datasheet, DIFF_FIFO_[9:0] is the number of unread FIFO *words*,
     *   where one FIFO "word" is 7 bytes: 1 TAG + 6 DATA bytes.
     *
     * This API returns BYTES (not frames) because fifoRead() and most callers
     * allocate raw byte buffers.
     */
    uint16_t frames = (uint16_t)((((uint16_t)(status[1] & 0x03)) << 8) | status[0]);
    level = (uint16_t)(frames * FIFO_TAG_FRAME_BYTES);

    return true;
}

/**
 * accelGPerLSB
 * - Returns accelerometer sensitivity in g/LSB for the current full-scale mode.
 * - Values are typical sensitivities (datasheet).
 */
float ISM330DHCX_7Semi::accelGPerLSB(ISM330_XL_FS xl_fs) const
{
    /**
     * Typical sensitivities (mg/LSB):
     * - ±2g  : 0.061
     * - ±4g  : 0.122
     * - ±8g  : 0.244
     * - ±16g : 0.488
     *
     * Converted to g/LSB by dividing mg by 1000.
     */
    switch (xl_fs)
    {
    case XL_FS_2G:
        return 0.000061f;
    case XL_FS_4G:
        return 0.000122f;
    case XL_FS_8G:
        return 0.000244f;
    case XL_FS_16G:
        return 0.000488f;
    default:
        return 0.000061f;
    }
}

/**
 * gyroDpsPerLSB
 * - Returns gyroscope sensitivity in dps/LSB for the current full-scale mode.
 * - Values are typical sensitivities (datasheet).
 */
float ISM330DHCX_7Semi::gyroDpsPerLSB(ISM330_G_FS g_fs) const
{
    /**
     * Typical sensitivities (mdps/LSB):
     * - 125  : 4.375
     * - 250  : 8.75
     * - 500  : 17.50
     * - 1000 : 35.0
     * - 2000 : 70.0
     * - 4000 : 140.0
     *
     * Converted to dps/LSB by dividing mdps by 1000.
     */
    switch (g_fs)
    {
    case G_FS_125:
        return 0.004375f;
    case G_FS_250:
        return 0.00875f;
    case G_FS_500:
        return 0.01750f;
    case G_FS_1000:
        return 0.03500f;
    case G_FS_2000:
        return 0.07000f;
    case G_FS_4000:
        return 0.14000f;
    default:
        return 0.00875f;
    }
}

/**
 * setAccelConfig
 * - Writes CTRL1_XL based on:
 *   - odr: output data rate (placed in bits [7:4])
 *   - fs : full-scale (placed in bits [3:2])
 *   - lpf2: enables LPF2 option (bit1 in this library mapping)
 *
 * Note:
 * - This function only writes the register; it does not update cached xl_fs/xl_odr unless you do that in the header/elsewhere.
 */
bool ISM330DHCX_7Semi::setAccelConfig(ISM330_XL_ODR odr, ISM330_XL_FS fs, bool lpf2)
{
    uint8_t v = 0;
    v |= (uint8_t)(odr << 4);
    v |= (uint8_t)(fs << 2);
    if (lpf2)
        v |= 1u << 1;

    if (!writeReg(CTRL1_XL, v))
        return false;
    // Serial.println(fs);
    accel_sensitivity = accelGPerLSB(fs); // Update sensitivity based on new fs
    return true;
}

/**
 * setGyroConfig
 * - Writes CTRL2_G based on:
 *   - odr: output data rate (placed in bits [7:4])
 *   - fs : full-scale selection (placed as defined by enum)
 *   - lpf1: enables LPF1 option (bit2 in this library mapping)
 */
bool ISM330DHCX_7Semi::setGyroConfig(ISM330_G_ODR odr, ISM330_G_FS fs, bool lpf1)
{
    uint8_t v = 0;
    v |= (uint8_t)(odr << 4);
    v |= (uint8_t)fs;

    if (!writeReg(CTRL2_G, v))
        return false;
    gyro_sensitivity = gyroDpsPerLSB(fs); // Update sensitivity based on new fs
    return writeBit(CTRL4_C, 1, lpf1);    // Set LPF1_SEL_G bit to enable LPF1 path
}

/**
 * enableBDU
 * - Toggles Block Data Update.
 * - Recommended ON when reading multi-byte outputs (accel/gyro/temp) to avoid torn reads.
 */
bool ISM330DHCX_7Semi::enableBDU(bool en)
{
    return writeBit(CTRL3_C, 6, en);
}

bool ISM330DHCX_7Semi::setAutoIncrement(bool enable)
{
    return writeBit(CTRL3_C, 2, enable);
}
/**
 * enableBDU
 * - Toggles Block Data Update.
 * - Recommended ON when reading multi-byte outputs (accel/gyro/temp) to avoid torn reads.
 */
bool ISM330DHCX_7Semi::memoryReboot()
{
    return writeBit(CTRL3_C, 7, true); // Set MEM_BOOT bit
}
/**
 * softwareReset
 * - Triggers device software reset.
 * - Caller should delay afterwards to allow reboot.
 */
bool ISM330DHCX_7Semi::softwareReset()
{
    return writeReg(CTRL3_C, CTRL3_SW_RESET);
}

/**
 * setGyroLPF1
 * - Selects gyro LPF1 bandwidth via CTRL6_C FTYPE bits.
 * - Enables LPF1 selection path via CTRL4_C LPF1_SEL_G.
 */
bool ISM330DHCX_7Semi::setGyroLPF1(ISM330_G_LPF1 bw)
{
    uint8_t v;
    readReg(CTRL6_C, v);
    v = (v & ~CTRL6_FTYPE_MASK) | (bw & 0x07);
    writeReg(CTRL6_C, v);

    return writeBit(CTRL4_C, 1, true); // Set LPF1_SEL_G bit to enable LPF1 path
}

/**
 * setAccelFilter
 * - Configures accelerometer high-pass / slope filter selection in CTRL8_XL.
 * - hpcf selects cutoff option (placed in bits [7:5]).
 * - highPass enables the HP/slope path.
 */
bool ISM330DHCX_7Semi::setAccelFilter(uint8_t hpcf, bool highPass)
{
    uint8_t v;
    readReg(CTRL8_XL, v);

    v &= ~(CTRL8_HPCF_MASK | CTRL8_HP_SLOPE_EN);
    v |= (hpcf << 5);
    if (highPass)
        v |= CTRL8_HP_SLOPE_EN;

    return writeReg(CTRL8_XL, v);
}

/**
 * setAccelODR
 * - Updates only the ODR field in CTRL1_XL.
 * - Preserves other CTRL1_XL bits (FS, LPF2 enable, etc.).
 */
bool ISM330DHCX_7Semi::setAccelODR(ISM330_XL_ODR odr)
{
    uint8_t v;
    if (!readReg(CTRL1_XL, v))
        return false;

    v &= ~XL_ODR_MASK;
    v |= (odr & XL_ODR_MASK);

    if (!writeReg(CTRL1_XL, v))
        return false;
    return true;
}

/**
 * setAccelFS
 * - Updates only the full-scale field in CTRL1_XL.
 * - Caller should also update xl_fs cached state if your header uses it for scaling.
 */
bool ISM330DHCX_7Semi::setAccelFS(ISM330_XL_FS fs)
{
    uint8_t v;
    if (!readReg(CTRL1_XL, v))
        return false;

    v &= ~XL_FS_MASK;
    v |= (fs & XL_FS_MASK);

    if (!writeReg(CTRL1_XL, v))
        return false;
    accel_sensitivity = accelGPerLSB(fs); // Update sensitivity based on new fs
    return true;
}

/**
 * setAccelLPF2
 * - Enables/disables accelerometer LPF2 bit in CTRL1_XL.
 */
bool ISM330DHCX_7Semi::setAccelLPF2(bool enable)
{
    return writeBit(CTRL1_XL, 1, enable); // Set LPF2_EN bit
}

/**
 * setGyroODR
 * - Updates only the ODR field in CTRL2_G.
 */
bool ISM330DHCX_7Semi::setGyroODR(ISM330_G_ODR odr)
{
    uint8_t v;
    if (!readReg(CTRL2_G, v))
        return false;

    v &= ~G_ODR_MASK;
    v |= (odr & G_ODR_MASK);

    return writeReg(CTRL2_G, v);
}

/**
 * setGyroFS
 * - Updates gyro FS bits in CTRL2_G.
 * - Also clears explicit 125 dps and 4000 dps flags before setting a new range.
 */
bool ISM330DHCX_7Semi::setGyroFS(ISM330_G_FS fs)
{
    uint8_t v;
    if (!readReg(CTRL2_G, v))
        return false;

    v &= ~(G_FS_MASK | G_FS_125 | G_FS_4000);
    v |= (fs & G_FS_MASK);

    if (!writeReg(CTRL2_G, v))
        return false;
    gyro_sensitivity = gyroDpsPerLSB(fs); // Update sensitivity based on new fs
    return true;
}
/* ========================= FUNCTION ACCESS ========================= */

/**
 * enableFuncCfgAccess
 * - Enables access to embedded function configuration registers.
 * - Uses FUNC_CFG_ACCESS register gating.
 */
bool ISM330DHCX_7Semi::enableFuncCfgAccess(bool enable)
{
    return writeBit(FUNC_CFG_ACCESS, 1, enable);
}

/**
 * enableShubAccess
 * - Enables sensor hub register access through FUNC_CFG_ACCESS gating.
 * - Required when configuring external sensors through the IMU sensor hub.
 */
bool ISM330DHCX_7Semi::enableShubAccess(bool enable)
{
    return writeBit(FUNC_CFG_ACCESS, 2, enable);
}

/* ========================= PIN CONTROL ========================= */

/**
 * pinCtrlEnableSdoPullup
 * - Controls internal pull-up on SDO line (bit1 in PIN_CTRL).
 * - Useful for I2C address strap stability or floating-line protection.
 */
bool ISM330DHCX_7Semi::pinCtrlEnableSdoPullup(bool enable)
{
    return writeBit(PIN_CTRL, 1, enable);
}

/**
 * pinCtrlDisableOcsSdoAuxPullup
 * - Controls OCS/SDO/AUX pull-up disable behavior (bit2 in PIN_CTRL).
 * - Comment in code says "must be 1 for correct operation" for your board/use-case.
 */
bool ISM330DHCX_7Semi::pinCtrlDisableOcsSdoAuxPullup(bool disable)
{
    return writeBit(PIN_CTRL, 2, disable);
}

/* ========================= BATCH COUNTER ========================= */

/**
 * setBatchCounterThreshold
 * - Programs batch counter threshold (0..1023).
 * - Threshold high bits go into COUNTER_BDR_REG1, low byte into COUNTER_BDR_REG2.
 */
bool ISM330DHCX_7Semi::setBatchCounterThreshold(uint16_t threshold)
{
    if (threshold > 1023)
        return false;

    uint8_t th_high = (threshold >> 8) & 0x07;
    uint8_t th_low = threshold & 0xFF;

    uint8_t reg1;
    if (!readReg(COUNTER_BDR_REG1, reg1))
        return false;

    reg1 &= ~((CNT_BDR_TH_10 | CNT_BDR_TH_9 | CNT_BDR_TH_8));
    reg1 |= (th_high << 2);

    if (!writeReg(COUNTER_BDR_REG1, reg1))
        return false;
    if (!writeReg(COUNTER_BDR_REG2, th_low))
        return false;

    return true;
}

/**
 * getBatchCounterThreshold
 * - Reads back the programmed batch counter threshold value.
 */
bool ISM330DHCX_7Semi::getBatchCounterThreshold(uint16_t &threshold)
{
    uint8_t reg1, reg2;
    if (!readReg(COUNTER_BDR_REG1, reg1))
        return false;
    if (!readReg(COUNTER_BDR_REG2, reg2))
        return false;

    uint16_t th_high = (reg1 >> 2) & 0x07;
    uint16_t th_low = reg2;

    threshold = bytesToInt16(th_low, th_high);
    return true;
}

/**
 * enablePulsedDR
 * - Enables/disables pulsed data-ready behavior for BDR counter logic.
 */
bool ISM330DHCX_7Semi::enablePulsedDR(bool enable)
{

    return writeBit(COUNTER_BDR_REG1, 7, enable); // Set DATAREADY_PULSED bit
}

/**
 * setBatchTrigger
 * - Selects trigger source for batch counter (gyro vs accel) using TRIG_COUNTER_BDR bit.
 */
bool ISM330DHCX_7Semi::setBatchTrigger(bool gyro)
{

    return writeBit(COUNTER_BDR_REG1, 5, gyro); // Set TRIG_COUNTER_BDR bit for gyro trigger
}

/**
 * resetBatchCounter
 * - Resets the batch counter using RST_COUNTER_BDR bit.
 */
bool ISM330DHCX_7Semi::resetBatchCounter()
{
    return writeBit(COUNTER_BDR_REG1, 6, true); // Set RST_COUNTER_BDR bit to reset counter
}

/* ========================= ACCEL/GYRO CONFIG ========================= */

/* ========================= CTRL5_C ========================= */

/**
 * setRounding
 * - Controls register data rounding behavior via CTRL5_C.
 * - Useful for aligning data reads to specific sensor sets (depends on your mode usage).
 */
bool ISM330DHCX_7Semi::setRounding(ISM330_Rounding mode)
{
    uint8_t v;
    if (!readReg(CTRL5_C, v))
        return false;

    v &= ~CTRL5_ROUNDING_MASK;
    v |= ((mode & 0x03) << 5);

    return writeReg(CTRL5_C, v);
}

/**
 * setGyroSelfTest
 * - Enables gyro self-test excitation.
 * - mode must be allowed (ST_NOT_ALLOWED rejected).
 */
bool ISM330DHCX_7Semi::setGyroSelfTest(ISM330_SelfTest mode)
{
    if (mode == ST_NOT_ALLOWED)
        return false;

    uint8_t v;
    if (!readReg(CTRL5_C, v))
        return false;

    v &= ~CTRL5_ST_G_MASK;
    v |= ((mode & 0x03) << 3);

    return writeReg(CTRL5_C, v);
}

/**
 * setAccelSelfTest
 * - Enables accel self-test excitation.
 * - mode must be allowed (ST_NOT_ALLOWED rejected).
 */
bool ISM330DHCX_7Semi::setAccelSelfTest(ISM330_SelfTest mode)
{
    if (mode == ST_NOT_ALLOWED)
        return false;

    uint8_t v;
    if (!readReg(CTRL5_C, v))
        return false;

    v &= ~CTRL5_ST_XL_MASK;
    v |= ((mode & 0x03) << 1);

    return writeReg(CTRL5_C, v);
}

/* ========================= CTRL6_C ========================= */

/**
 * setTriggerMode
 * - Configures trigger behavior using CTRL6_C bits.
 * - Different modes set TRIG_EN and LVL1/LVL2 combinations.
 */
bool ISM330DHCX_7Semi::setTriggerMode(ISM330_TrigMode mode)
{
    uint8_t v;
    if (!readReg(CTRL6_C, v))
        return false;

    v &= ~(CTRL6_TRIG_EN | CTRL6_LVL1_EN | CTRL6_LVL2_EN);

    switch (mode)
    {
    case TRIG_EDGE:
        v |= CTRL6_TRIG_EN;
        break;
    case TRIG_LEVEL:
        v |= CTRL6_LVL1_EN;
        break;
    case TRIG_LATCHED:
        v |= (CTRL6_LVL1_EN | CTRL6_LVL2_EN);
        break;
    case TRIG_FIFO_ENABLE:
        v |= (CTRL6_TRIG_EN | CTRL6_LVL1_EN);
        break;
    default:
        return false;
    }

    return writeReg(CTRL6_C, v);
}

/**
 * setAccelHighPerformance
 * - Controls accelerometer high-performance vs high-power mode via CTRL6_C.
 * - enable=true clears HM bit (high performance).
 * - enable=false sets HM bit (low-power/high-efficiency).
 */
bool ISM330DHCX_7Semi::setAccelHighPerformance(bool enable)
{
    uint8_t v;
    if (!readReg(CTRL6_C, v))
        return false;

    if (enable)
        v &= ~CTRL6_XL_HM_MODE;
    else
        v |= CTRL6_XL_HM_MODE;

    return writeReg(CTRL6_C, v);
}

/**
 * setAccelOffsetWeight
 * - Selects user offset weight mode via CTRL6_C.
 * - Used when user offset correction is enabled/used.
 */
bool ISM330DHCX_7Semi::setAccelOffsetWeight(bool highWeight)
{
    uint8_t v;
    if (!readReg(CTRL6_C, v))
        return false;

    if (highWeight)
        v |= CTRL6_USR_OFF_W;
    else
        v &= ~CTRL6_USR_OFF_W;

    return writeReg(CTRL6_C, v);
}

/**
 * setGyroLPF1Bandwidth
 * - Writes gyro LPF1 bandwidth selection field (FTYPE) in CTRL6_C.
 * - ftype is limited to 3-bit value (0..7).
 */
bool ISM330DHCX_7Semi::setGyroLPF1Bandwidth(uint8_t ftype)
{
    if (ftype > 0x07)
        return false;

    uint8_t v;
    if (!readReg(CTRL6_C, v))
        return false;

    v &= ~CTRL6_FTYPE_MASK;
    v |= (ftype & 0x07);

    return writeReg(CTRL6_C, v);
}

/* ========================= CTRL7_G ========================= */

/**
 * setGyroHighPerf
 * - Controls gyroscope high-performance vs low-power mode via CTRL7_G.
 */
bool ISM330DHCX_7Semi::setGyroHighPerf(bool enable)
{

    return writeBit(CTRL7_G, 7, enable); // Set G_HM_MODE bit to enable high-performance mode
}

/**
 * enableGyroHPF
 * - Enables/disables gyro high-pass filter path via CTRL7_G.
 */
bool ISM330DHCX_7Semi::enableGyroHPF(bool enable)
{
    return writeBit(CTRL7_G, 6, enable); // Set HP_EN_G bit to enable gyro HPF path
}

/**
 * setGyroHPFCutoff
 * - Selects gyro HPF cutoff option (2-bit value).
 */
bool ISM330DHCX_7Semi::setGyroHPFCutoff(ISM330_G_HP_CUTOFF cutoff)
{
    if (cutoff > 3)
        return false;

    uint8_t v;
    if (!readReg(CTRL7_G, v))
        return false;

    v &= ~CTRL7_HPM_G_MASK;
    v |= ((cutoff & 0x03) << 4);

    return writeReg(CTRL7_G, v);
}

/**
 * enableOIS
 * - Enables/disables OIS path and selects whether primary interface controls it.
 * - Useful in camera stabilization style use-cases.
 */
bool ISM330DHCX_7Semi::enableOIS(bool enable, bool controlViaPrimary)
{
    // OIS_ON_EN → bit 2
    if (!writeBit(CTRL7_G, 2, controlViaPrimary))
        return false;

    // OIS_ON → bit 0
    return writeBit(CTRL7_G, 0, enable);
}

/**
 * enableAccelUserOffset
 * - Enables user offset output correction (when offsets are configured).
 */
bool ISM330DHCX_7Semi::enableAccelUserOffset(bool enable)
{
    uint8_t v;
    if (!readReg(CTRL7_G, v))
        return false;

    if (enable)
        v |= CTRL7_USR_OFF_ON_OUT;
    else
        v &= ~CTRL7_USR_OFF_ON_OUT;

    return writeReg(CTRL7_G, v);
}

/* ========================= CTRL8_XL ========================= */

/**
 * setAccelLowPass
 * - Selects accelerometer LP/HP cutoff configuration in CTRL8_XL.
 * - hpcf is 3-bit (0..7) and mapped into bits [7:5].
 */
bool ISM330DHCX_7Semi::setAccelLowPass(uint8_t hpcf)
{
    if (hpcf > 7)
        return false;

    uint8_t v;
    if (!readReg(CTRL8_XL, v))
        return false;

    v &= ~CTRL8_HPCF_MASK;
    v |= (hpcf << 5);

    return writeReg(CTRL8_XL, v);
}

/**
 * enableAccelHPF
 * - Enables/disables accel HP/slope filter path via CTRL8_XL.
 */
bool ISM330DHCX_7Semi::enableAccelHPF(bool enable)
{

    return writeBit(CTRL8_XL, 2, enable); // Set HP_SLOPE_EN bit to enable accel HPF path
}

/**
 * enableAccelHPRef
 * - Enables/disables high-pass reference mode (depends on selected filter path).
 */
bool ISM330DHCX_7Semi::enableAccelHPRef(bool enable)
{

    return writeBit(CTRL8_XL, 4, enable); // Set HP_REF_MODE bit to enable/disable accel HP reference mode
}

/**
 * enableAccelFastSettle
 * - Enables fast settling for accel filters.
 * - Can reduce start-up transient time in some modes.
 */
bool ISM330DHCX_7Semi::enableAccelFastSettle(bool enable)
{
    return writeBit(CTRL8_XL, 3, enable); // Set FAST_SETTL bit to enable/disable fast settling mode
}

/**
 * enableLowPassOn6D
 * - Enables low-pass filter on 6D orientation detection path.
 */
bool ISM330DHCX_7Semi::enableLowPassOn6D(bool enable)
{
    return writeBit(CTRL8_XL, 0, enable); // Set LOW_PASS_ON_6D bit to enable/disable low-pass filter on 6D path
}

/**
 * enableDeviceConfigure
 * - Toggles DEVICE_CONF bit in CTRL9_XL (bit1 in this library mapping).
 * - Some advanced features require this to be set according to the datasheet sequence.
 */
bool ISM330DHCX_7Semi::enableDeviceConfigure(bool enable)
{
    return writeBit(CTRL9_XL, 1, enable); // Set DEVICE_CONF bit to enable/disable device configuration mode
}

bool ISM330DHCX_7Semi::enableTimestamp(bool enable)
{
    return writeBit(CTRL10_C, 5, enable);
}

/* ========================= SMOOTHING ========================= */

/**
 * smoothUpdate
 * - Simple 1D Kalman-like smoother.
 * - s.q : process noise (how quickly you expect signal can change)
 * - s.r : measurement noise (how noisy your sensor reading is)
 * - s.p : estimation error covariance
 * - s.x : filtered output
 *
 * Practical guidance:
 * - Larger r => more smoothing (slower response).
 * - Larger q => faster response (less smoothing).
 */
float ISM330DHCX_7Semi::smoothUpdate(Smooth1D &s, float measurement)
{
    if (!s.init)
    {
        s.x = measurement;
        s.init = true;
        return s.x;
    }

    s.p = s.p + s.q;

    float k = s.p / (s.p + s.r);

    s.x = s.x + k * (measurement - s.x);
    s.p = (1.0f - k) * s.p;

    return s.x;
}

size_t ISM330DHCX_7Semi::parseFifoTaggedFrames(const uint8_t *buf,
                                               size_t bytes,
                                               fifoSample *outSamples,
                                               size_t maxSamples)
{
    if (!buf || !outSamples || maxSamples == 0)
        return 0;

    /**
     * - Only parse full frames
     * - Tagged frame is fixed-size (TAG + X/Y/Z 16-bit)
     */
    const size_t frames = bytes / FIFO_TAG_FRAME_BYTES;
    size_t written = 0;

    for (size_t i = 0; i < frames && written < maxSamples; i++)
    {
        const uint8_t *f = &buf[i * FIFO_TAG_FRAME_BYTES];

        /**
         * - ST tag is typically stored in bits [7:3]
         * - Keep only 5-bit tag ID
         */
        const uint8_t tag = (uint8_t)((f[0] >> 3) & 0x1F);

        const int16_t x = bytesToInt16(f[1], f[2]);
        const int16_t y = bytesToInt16(f[3], f[4]);
        const int16_t z = bytesToInt16(f[5], f[6]);

        fifoSample s{};
        s.newGyro = false;
        s.newAccel = false;
        s.newTemp = false;

        switch (tag)
        {
        case 0x01: /** - Gyro tagged sample */
            s.rawGx = x;
            s.rawGy = y;
            s.rawGz = z;
            s.gx = (float)s.rawGx * gyro_sensitivity;
            s.gy = (float)s.rawGy * gyro_sensitivity;
            s.gz = (float)s.rawGz * gyro_sensitivity;
            s.newGyro = true;
            break;

        case 0x02: /** - Accel tagged sample */
            s.rawAx = x;
            s.rawAy = y;
            s.rawAz = z;
            s.ax = (float)s.rawAx * accel_sensitivity;
            s.ay = (float)s.rawAy * accel_sensitivity;
            s.az = (float)s.rawAz * accel_sensitivity;
            s.newAccel = true;
            break;

        default:
            /**
             * - Ignore unknown tags safely
             * - Do not count as a written sample
             */
            continue;
        }

        outSamples[written++] = s;
    }

    return written;
}

bool ISM330DHCX_7Semi::fifoReadSample(fifoSample &sample)
{
    // sample = fifoSample{}; // Optional: zero out the sample struct before filling (depends on your use-case)

    /**
     * - Read exactly one tagged frame (7 bytes)
     * - Use uint8_t, not int8_t (we are reading raw bytes)
     */
    uint8_t frame[FIFO_TAG_FRAME_BYTES]; //= {0};

    if (!fifoRead(frame, FIFO_TAG_FRAME_BYTES))
        return false;
    int16_t fx, fy, fz;
    const uint8_t tag = (uint8_t)((frame[0] >> 3) & 0x1F);

    switch (tag)
    {
    case 0x01: /** - Gyro */
        fx = bytesToInt16(frame[1], frame[2]);
        fy = bytesToInt16(frame[3], frame[4]);
        fz = bytesToInt16(frame[5], frame[6]);

        if (smoothEnable)
        {
            fx = smoothUpdate(sgx, fx);
            fy = smoothUpdate(sgy, fy);
            fz = smoothUpdate(sgz, fz);
        }
        sample.rawGx = fx;
        sample.rawGy = fy;
        sample.rawGz = fz;
        sample.gx = (float)sample.rawGx * gyro_sensitivity;
        sample.gy = (float)sample.rawGy * gyro_sensitivity;
        sample.gz = (float)sample.rawGz * gyro_sensitivity;
        sample.newGyro = true;
        return true;

    case 0x02: /** - Accel */
        fx = bytesToInt16(frame[1], frame[2]);
        fy = bytesToInt16(frame[3], frame[4]);
        fz = bytesToInt16(frame[5], frame[6]);
        if (smoothEnable)
        {
            fx = smoothUpdate(sax, fx);
            fy = smoothUpdate(say, fy);
            fz = smoothUpdate(saz, fz);
        }
        sample.rawAx = fx;
        sample.rawAy = fy;
        sample.rawAz = fz;
        sample.ax = (float)sample.rawAx * accel_sensitivity;
        sample.ay = (float)sample.rawAy * accel_sensitivity;
        sample.az = (float)sample.rawAz * accel_sensitivity;
        sample.newAccel = true;
        return true;

    case 0x03: /** - Temp (only if you actually enabled temp batching) */
        /**
         * - Temp isn’t normally packed as XYZ.
         * - Keep this only if your FIFO config pushes temp in this tag format.
         * - Otherwise delete this case to avoid fake temp results.
         */
        sample.rawTemp = bytesToInt16(frame[1], frame[2]);
        sample.temp = ((float)sample.rawTemp * temperature_sensitivity) + 25.0f;
        sample.newTemp = true;
        return true;

    default:
        /**
         * - Unknown tag
         * - Return false so caller can ignore this frame
         */
        return false;
    }
}

bool ISM330DHCX_7Semi::interruptPolarity(bool activeHigh)
{
    return writeBit(CTRL3_C, 5, !activeHigh);
}

bool ISM330DHCX_7Semi::clearInterrupts()
{
    writeBit(TAP_CFG0, 6, true); // Set TAP_CLEAR_INT bit to clear interrupts immediately
}

bool ISM330DHCX_7Semi::getNewDataStatus(bool &accelReady, bool &gyroReady, bool &tempReady)
{
    uint8_t v;
    if (!readReg(DRD_SRC, v))
        return false;

    accelReady = (v & (1 << 0)) != 0;
    gyroReady = (v & (1 << 1)) != 0;
    tempReady = (v & (1 << 2)) != 0;

    return true;
}
