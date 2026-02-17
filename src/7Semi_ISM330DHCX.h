/**
 * 7Semi ISM330DHCX 
 * - Arduino-friendly driver for ST ISM330DHCX IMU (Accel + Gyro + Temp).
 * - Supports I2C through the 7Semi BusDriver abstraction.
 * - This header defines:
 *   - Public API for initialization, sensor reads, FIFO, and configuration
 *   - Enums for ODR, full-scale, and filter options
 *   - Data structures used for FIFO parsing (fifoSample)
 */
#pragma once

#include <Arduino.h>
#include "7Semi_BusDriver.h"
#include "7Semi_ISM330DHCX_defs.h"

/* ========================= ENUMS ========================= */

/**
 * ISM330_G_ODR
 * - Gyroscope Output Data Rate selection values.
 * - These values map to the CTRL2_G ODR field (bits [7:4]) via shifting in setGyroConfig().
 * - Use G_ODR_OFF to power down gyro output.
 */
enum ISM330_G_ODR : uint8_t
{
    G_ODR_OFF = 0x00,
    G_ODR_12_5Hz = 0x01,
    G_ODR_26Hz = 0x02,
    G_ODR_52Hz = 0x03,
    G_ODR_104Hz = 0x04,
    G_ODR_208Hz = 0x05,
    G_ODR_416Hz = 0x06,
    G_ODR_833Hz = 0x07,
    G_ODR_1660Hz = 0x08,
    G_ODR_3330Hz = 0x09,
    G_ODR_6660Hz = 0x0A
};

/**
 * ISM330_XL_ODR
 * - Accelerometer Output Data Rate selection values.
 * - These values map to the CTRL1_XL ODR field (bits [7:4]) via shifting in setAccelConfig().
 * - XL_ODR_1_6Hz is a special low-rate mode supported by this device family.
 */
enum ISM330_XL_ODR : uint8_t
{
    XL_ODR_OFF = 0x00,
    XL_ODR_12_5Hz = 0x01,
    XL_ODR_26Hz = 0x02,
    XL_ODR_52Hz = 0x03,
    XL_ODR_104Hz = 0x04,
    XL_ODR_208Hz = 0x05,
    XL_ODR_416Hz = 0x06,
    XL_ODR_833Hz = 0x07,
    XL_ODR_1660Hz = 0x08,
    XL_ODR_3330Hz = 0x09,
    XL_ODR_6660Hz = 0x0A,
    XL_ODR_1_6Hz = 0x0B
};

/**
 * ISM330_XL_FS
 * - Accelerometer full-scale selection values.
 * - These are encoded for placement into CTRL1_XL FS field (bits [3:2]) by shifting in setAccelConfig().
 * - Naming here uses trailing underscore; keep consistent with the rest of your codebase.
 */
enum ISM330_XL_FS : uint8_t
{
    XL_FS_2G = 0,
    XL_FS_4G = 2,
    XL_FS_8G = 3,
    XL_FS_16G = 1
};

/**
 * ISM330_G_FS
 * - Gyroscope full-scale selection values.
 * - These values map into CTRL2_G FS field (bits [3:2]) plus special-case flags for 125 dps and 4000 dps.
 * - G_FS_125 and G_FS_4000 are special encodings on this device family.
 */
enum ISM330_G_FS : uint8_t
{
    G_FS_250 = 0x00,
    G_FS_500 = 0x04,
    G_FS_1000 = 0x08,
    G_FS_2000 = 0x0C,
    G_FS_125 = 0x02,
    G_FS_4000 = 0x01
};

/**
 * ISM330_G_LPF1
 * - Gyro LPF1 bandwidth selection.
 * - Intended to be written into CTRL6_C FTYPE bits (masking handled in setGyroLPF1()).
 */
enum ISM330_G_LPF1 : uint8_t
{
    G_LPF1_0 = 0,
    G_LPF1_1,
    G_LPF1_2,
    G_LPF1_3,
    G_LPF1_4,
    G_LPF1_5,
    G_LPF1_6,
    G_LPF1_7
};

/**
 * ISM330_Rounding
 * - Output rounding behavior selection.
 * - Used by setRounding() to control CTRL5_C rounding field.
 */
enum ISM330_Rounding : uint8_t
{
    ROUNDING_NONE = 0,
    ROUNDING_XL = 1,
    ROUNDING_G = 2,
    ROUNDING_BOTH = 3
};

/**
 * ISM330_SelfTest
 * - Generic self-test selection used by some APIs.
 * - ST_NOT_ALLOWED can be used by higher layers to reject unsupported modes.
 */
enum ISM330_SelfTest : uint8_t
{
    ST_NORMAL = 0,
    ST_POSITIVE = 1,
    ST_NEGATIVE = 2,
    ST_NOT_ALLOWED = 3
};

enum IntCtrlMask : uint8_t
{
    /** - Data-ready accel */
    INT_DRDY_XL = 1 << 0,
    /** - Data-ready gyro */
    INT_DRDY_G = 1 << 1,
    /** - Boot status available */
    INT_BOOT = 1 << 2,
    /** - FIFO watermark */
    INT_FIFO_TH = 1 << 3,
    /** - FIFO overrun */
    INT_FIFO_OVR = 1 << 4,
    /** - FIFO full */
    INT_FIFO_FULL = 1 << 5,
    /** - Batch counter threshold */
    INT_CNT_BDR = 1 << 6
};

enum MdCfgMask : uint8_t
{
    /** - Sensor hub / external sensor event */
    MD_INT_SHUB = 1 << 0,
    /** - 6D orientation */
    MD_INT_6D = 1 << 1,
    /** - Single tap */
    MD_INT_SINGLE_TAP = 1 << 2,
    /** - Wake-up */
    MD_INT_WU = 1 << 3,
    /** - Free-fall */
    MD_INT_FF = 1 << 4,
    /** - Double tap */
    MD_INT_DOUBLE_TAP = 1 << 5,
    /** - Sleep change */
    MD_INT_SLEEP_CHANGE = 1 << 6,
    /** - Embedded functions (FSM/MLC/etc) */
    MD_INT_EMB_FUNC = 1 << 7
};

struct InterruptSources
{
    /** - STATUS_REG */
    bool drdyXL;
    bool drdyG;
    bool drdyTemp;

    /** - FIFO_STATUS2 */
    bool fifoWatermark;
    bool fifoOverrun;
    bool fifoFull;

    /** - ALL_INT_SRC */
    bool wakeUp;
    bool freeFall;
    bool tap;
    bool d6d;
    bool sleepChange;

    /** - WAKE_UP_SRC / TAP_SRC / D6D_SRC raw */
    uint8_t wakeUpSrc;
    uint8_t tapSrc;
    uint8_t d6dSrc;

    /** - Embedded functions status (main page) */
    uint8_t embFuncStatus;
};
/**
 * fifoSample
 * - Convenience structure for FIFO/sample parsing workflows.
 * - newGyro/newGyro allow partial frames or mixed FIFO tags to be represented safely.
 */
struct fifoSample
{
    int16_t rawGx, rawGy, rawGz;
    int16_t rawAx, rawAy, rawAz;
    int16_t rawTemp;
    float gx, gy, gz;
    float ax, ay, az;
    float temp;
    bool newGyro;
    bool newAccel;
    bool newTemp;
};

/* ========================= CLASS ========================= */

/**
 * ISM330DHCX_7Semi
 * - Driver class for ISM330DHCX-family IMU.
 * - Supports both I2C and SPI via a shared BusDriver abstraction.
 * - Provides:
 *   - Basic init + identity check
 *   - Accel/gyro/temp reads
 *   - FIFO controls
 *   - Interrupt routing helpers
 *   - Filter / self-test / advanced configuration helpers
 */
class ISM330DHCX_7Semi
{
public:
    ISM330DHCX_7Semi();

    /* ---- Bus init ---- */

    /**
     * beginI2C
     * - Initializes bus for I2C.
     * - addr defaults to 0x6A (common ISM330DHCX SA0=0 strap).
     * - clockHz defaults to 400k for fast-mode operation.
     */
    bool beginI2C(TwoWire &wire, uint8_t addr = 0x6A, uint32_t clockHz = 400000);

    /**
     * beginSPI
     * - Initializes bus for SPI.
     * - csPin is the chip-select pin used by the bus layer.
     * - clockHz defaults to 1 MHz (safe starting point).
     */
    bool beginSPI(SPIClass &spi, int csPin, uint32_t clockHz = 1000000);

    /**
     * begin
     * - Verifies device identity and applies default configuration.
     * - Enables BDU and auto-increment.
     * - Configures accel + gyro default ODR/FS/filter settings.
     */
    bool begin(TwoWire &wire, uint8_t addr = 0x6A, uint32_t clockHz = 400000);

    bool getProductID(uint8_t &id);

    /* ---- Sensor read ---- */

    /**
     * readAccel
     * - Reads accelerometer and returns values in g.
     * - Scaling uses accelGPerLSB() which depends on xl_fs.
     */
    bool readAccel(float &x, float &y, float &z);

    /**
     * readAccelMS2
     * - Reads accel in g and converts to m/s^2.
     */
    bool readAccelMS2(float &x, float &y, float &z);

    /**
     * readGyro
     * - Reads gyroscope and returns values in deg/s.
     * - Scaling uses gyroDpsPerLSB() which depends on g_fs.
     */
    bool readGyro(float &x, float &y, float &z);

    /**
     * readGyroRads
     * - Reads gyro in deg/s and converts to rad/s.
     */
    bool readGyroRads(float &x, float &y, float &z);

    /**
     * gyroDpsPerLSB
     * - Returns deg/s per LSB sensitivity for the given gyro full-scale setting.
     */
    float gyroDpsPerLSB(ISM330_G_FS g_fs) const;

    /**
     * accelGPerLSB
     * - Returns g/LSB sensitivity for the given accel full-scale setting.
     */
    float accelGPerLSB(ISM330_XL_FS a_fs) const;

    /**
     * readTemp
     * - Reads temperature sensor and returns °C.
     */
    bool readTemp(float &t);

    /* ---- FIFO ---- */

    /**
     * fifoEnable
     * - Enables FIFO with a chosen mode (default 0x06, library-specific).
     * - FIFO routing/decimation depends on FIFO_CTRLx configuration.
     */
    bool fifoEnable(uint8_t mode = 0x06);

    /**
     * fifoDisable
     * - Disables FIFO.
     */
    bool fifoDisable();

    /**
     * fifoSetWatermark
     * - Sets FIFO watermark threshold.
     * - Used for watermark interrupt / polling workflows.
     */
    bool fifoSetWatermark(uint16_t wtm);

    /**
     * fifoRead
     * - Reads available FIFO data into caller buffer.
     * - outLen returns how many bytes were read.
     */
    bool fifoRead(uint8_t *buf, uint16_t len);

    /**
     * fifoReady
     * - Returns internal FIFO-ready state.
     * - This flag must be managed by user code or ISR updates in your implementation.
     */
    bool fifoReady();

    /* ---- Interrupts ---- */

    /**
     * int1Enable / int1Disable
     * - Sets/clears routing bits in INT1_CTRL.
     * - mask must match bit definitions from defs header.
     */
    bool int1Enable(uint8_t mask);
    bool int1Disable(uint8_t mask);

    /**
     * int2Enable / int2Disable
     * - Sets/clears routing bits in INT2_CTRL.
     * - mask must match bit definitions from defs header.
     */
    bool int2Enable(uint8_t mask);
    bool int2Disable(uint8_t mask);

    /**
     * int1Read / int2Read
     * - Reads the current routing configuration registers.
     */
    bool int1Read(uint8_t &value);
    bool int2Read(uint8_t &value);

    /* ---- Configuration ---- */

    /**
     * setAccelConfig
     * - Writes ODR/FS and optional LPF2 enable into CTRL1_XL.
     */
    bool setAccelConfig(ISM330_XL_ODR odr, ISM330_XL_FS fs, bool lpf2);

    /**
     * setGyroConfig
     * - Writes ODR/FS and optional LPF1 enable into CTRL2_G.
     */
    bool setGyroConfig(ISM330_G_ODR odr, ISM330_G_FS fs, bool lpf1);

    /**
     * enableBDU
     * - Toggles BDU (Block Data Update) in CTRL3_C.
     * - Recommended ON for coherent multi-byte sensor reads.
     */
    bool enableBDU(bool en);
    bool memoryReboot();
    /**
     * softwareReset
     * - Triggers software reset (CTRL3_C SW_RESET).
     * - Caller should delay after this call.
     */
    bool softwareReset();

    /**
     * setGyroLPF1
     * - Configures gyro LPF1 bandwidth and selects LPF1 path.
     */
    bool setGyroLPF1(ISM330_G_LPF1 bw);

    /**
     * setAccelFilter
     * - Configures accel high-pass/slope filter behavior in CTRL8_XL.
     */
    bool setAccelFilter(uint8_t hpcf, bool highPass);

    /**
     * setAccelODR / setAccelFS / setAccelLPF2
     * - Read-modify-write helpers for CTRL1_XL fields.
     */
    bool setAccelODR(ISM330_XL_ODR odr);
    bool setAccelFS(ISM330_XL_FS fs);
    bool setAccelLPF2(bool enable);

    /**
     * setGyroODR / setGyroFS / setGyroFS125 / setGyroFS4000
     * - Read-modify-write helpers for CTRL2_G fields and special modes.
     */
    bool setGyroODR(ISM330_G_ODR odr);
    bool setGyroFS(ISM330_G_FS fs);
    bool enableGyroLPF1(bool en);

    /**
     * Misc control helpers (declared here, implemented elsewhere).
     * - These typically manipulate CTRL3_C/CTRL4_C/CTRL9_XL and related registers.
     */
    bool interruptPolarity(bool activeHigh);
    // bool setInterruptPolarity(bool activeLow);
    bool setInterruptOutputMode(bool openDrain);
    bool setSpiMode3Wire(bool en);
    bool setAutoIncrement(bool en);

    bool rebootMemory();

    bool enableGyroSleep(bool en);
    bool routeAllInterruptsToINT1(bool en);
    bool clearInterrupts();
    bool getNewDataStatus(bool &, bool &, bool &);
    bool enableDrdyMask(bool en);
    bool disableI2C(bool en);

    /**
     * ISM330_TrigMode
     * - Encodes trigger behavior selection for CTRL6_C.
     * - Mapping comments indicate bit patterns for TRIG/LVL fields.
     */
    enum ISM330_TrigMode : uint8_t
    {
        TRIG_EDGE = 0,       // 100
        TRIG_LEVEL = 1,      // 010
        TRIG_LATCHED = 2,    // 011
        TRIG_FIFO_ENABLE = 3 // 110
    };

    /**
     * setTriggerMode
     * - Applies the selected trigger mode in CTRL6_C.
     */
    bool setTriggerMode(ISM330_TrigMode mode);

    /**
     * setAccelHighPerformance
     * - Enables/disables accel high-performance mode (CTRL6_C).
     */
    bool setAccelHighPerformance(bool enable);

    /**
     * setAccelOffsetWeight
     * - Selects user offset weight mode (CTRL6_C).
     */
    bool setAccelOffsetWeight(bool highWeight);

    /**
     * setGyroLPF1Bandwidth
     * - Sets gyro LPF1 bandwidth field (FTYPE) directly.
     */
    bool setGyroLPF1Bandwidth(uint8_t ftype);

    /**
     * ISM330_G_HP_CUTOFF
     * - Gyro high-pass cutoff selection for CTRL7_G.
     */
    enum ISM330_G_HP_CUTOFF : uint8_t
    {
        G_HP_16mHz = 0,
        G_HP_65mHz = 1,
        G_HP_260mHz = 2,
        G_HP_1_04Hz = 3
    };

    bool setGyroHighPerf(bool enable);
    bool enableGyroHPF(bool enable);
    bool setGyroHPFCutoff(ISM330_G_HP_CUTOFF cutoff);

    /**
     * enableOIS
     * - Controls OIS path enable and whether primary interface controls it.
     */
    bool enableOIS(bool enable, bool controlViaPrimary = false);

    /**
     * enableAccelUserOffset
     * - Enables user offset correction output.
     */
    bool enableAccelUserOffset(bool enable);

    /**
     * ISM330_XL_FILTER
     * - Enumerates ODR-divider based filter options for accel.
     * - Use with your filter helper functions as needed.
     */
    enum ISM330_XL_FILTER : uint8_t
    {
        XL_FILTER_ODR_DIV_2, // default
        XL_FILTER_ODR_DIV_4,
        XL_FILTER_ODR_DIV_10,
        XL_FILTER_ODR_DIV_20,
        XL_FILTER_ODR_DIV_45,
        XL_FILTER_ODR_DIV_100,
        XL_FILTER_ODR_DIV_200,
        XL_FILTER_ODR_DIV_400,
        XL_FILTER_ODR_DIV_800
    };

    bool setAccelLowPass(uint8_t hpcf);
    bool enableAccelHPF(bool enable);
    bool enableAccelHPRef(bool enable);
    bool enableAccelFastSettle(bool enable);
    bool enableLowPassOn6D(bool enable);

    /* ---- Batch counter ---- */

    /**
     * Batch counter controls
     * - Used for batching + data-ready counting workflows.
     * - Threshold range is typically limited (implementation enforces 0..1023).
     */
    bool setBatchCounterThreshold(uint16_t threshold);
    bool getBatchCounterThreshold(uint16_t &threshold);
    bool enablePulsedDR(bool enable);
    bool setBatchTrigger(bool gyro);
    bool resetBatchCounter();

    /* ---- Function access gating ---- */

    /**
     * enableFuncCfgAccess
     * - Enables embedded function register access.
     */
    bool enableFuncCfgAccess(bool enable);

    /**
     * enableShubAccess
     * - Enables sensor hub register access.
     */
    bool enableShubAccess(bool enable);

    /* ---- Pin control ---- */

    /**
     * pinCtrlEnableSdoPullup
     * - Enables/disables SDO pull-up control via PIN_CTRL.
     */
    bool pinCtrlEnableSdoPullup(bool enable);

    /**
     * pinCtrlDisableOcsSdoAuxPullup
     * - Controls OCS/SDO/AUX pull-up disable behavior via PIN_CTRL.
     */
    bool pinCtrlDisableOcsSdoAuxPullup(bool disable);

    /* ---- Timestamp / device configure ---- */

    /**
     * enableTimestamp
     * - Enables timestamp counter output.
     */
    bool enableTimestamp(bool enable);

    /**
     * enableDeviceConfigure
     * - Toggles DEVICE_CONF bit used by some advanced configuration sequences.
     */
    bool enableDeviceConfigure(bool enable);

    /**
     * setRounding
     * - Controls output rounding behavior in CTRL5_C.
     */
    enum ISM330_Rounding : uint8_t
    {
        ROUNDING_NONE = 0,
        ROUNDING_XL = 1,
        ROUNDING_G = 2,
        ROUNDING_BOTH = 3
    };

    bool setRounding(ISM330_Rounding mode);

    /**
     * Self-test selection used by setGyroSelfTest()/setAccelSelfTest().
     * - Kept simple: OFF/POS/NEG only.
     */
    enum ISM330_SelfTest : uint8_t
    {
        SELFTEST_OFF = 0,
        SELFTEST_POS = 1,
        SELFTEST_NEG = 2
    };

    bool setGyroSelfTest(ISM330_SelfTest mode);
    bool setAccelSelfTest(ISM330_SelfTest mode);
   

    /* =========================
     * Add these library functions to your class
     * - fifoReadSample(fifoSample &sample)
     * - parseFifoTaggedFrames(...)
     *
     * This gives you a clean “library-style” FIFO read that returns one decoded sample at a time,
     * and a bulk parser that can fill many samples from a buffer.
     *
     * Assumption (matches your earlier commented code):
     * - FIFO outputs tagged frames of 7 bytes:
     *   - byte0: TAG (tag is in bits [7:3])
     *   - byte1..6: X/Y/Z as little-endian int16
     */

    /** Put these in 7Semi_ISM330DHCX.h (inside class public:) */

    /**
     * fifoReadSample
     * - Reads exactly one tagged FIFO frame and decodes it into fifoSample.
     * - Returns true if a valid accel or gyro sample was decoded.
     * - Returns false if FIFO is empty or frame tag is not accel/gyro.
     */
    bool fifoReadSample(fifoSample &sample);

    /**
     * parseFifoTaggedFrames
     * - Parses a buffer containing N tagged frames (7 bytes each).
     * - Appends decoded samples into outSamples (up to maxSamples).
     * - Returns how many samples were written.
     */
    size_t parseFifoTaggedFrames(const uint8_t *buf, size_t bytes, fifoSample *outSamples, size_t maxSamples);

    /** Put these in 7Semi_ISM330DHCX.h (inside class private:) */

    /** - Tagged FIFO frame size used by this library */
    static constexpr uint8_t FIFO_TAG_FRAME_BYTES = 7;

    /**
     * fifoGetLevel
     * - Reads FIFO unread level from FIFO_STATUS1/2.
     * - Returns number of available bytes to read from FIFO_DATA_OUT_TAG stream.
     * - If your silicon reports “words” instead of bytes, adjust the multiplier here once.
     */
    bool fifoGetLevel(uint16_t &level);

private:
    /**
     * bus
     * - Transport abstraction used for I2C/SPI operations.
     */
    BusDriver bus;

    /**
     * fifo_ready
     * - Internal flag indicating FIFO event/ready state.
     * - Must be managed by your FIFO ISR/poll logic in the .cpp.
     */
    bool fifo_ready;

    /**
     * Cached scaling configuration
     * - These cached values are used by accelGPerLSB()/gyroDpsPerLSB().
     * - Keep these updated whenever FS is changed (setAccelConfig/setGyroConfig or FS setters).
     */
    // uint8_t xl_fs = XL_FS_2G; // default to 2g FS for correct sensitivity calculation
    // uint8_t g_fs = G_FS_125;  // default to 125 dps FS (special case sensitivity)
    float accel_sensitivity = 0.000061f;         // g/LSB for 2g FS
    float gyro_sensitivity = 0.00875f;           // dps/LSB for 125 dps FS
    float temperature_sensitivity = 0.00390625f; // °C/LSB (example value, check datasheet)

    /**
     * writeReg
     * - Private register write helper used internally.
     */
    /**
     * readReg / readRegs
     * - Exposes low-level register access.
     * - Useful for debug or advanced tuning not covered by helper methods.
     */

    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t &data);
    bool readRegs(uint8_t reg, uint8_t *buf, size_t len);
    bool writeBit(uint8_t reg, uint8_t bit_position, bool value);
    bool getBit(uint8_t reg, uint8_t bit_position, bool &bit_value);

     /**
     * Smooth1D
     * - Small 1D Kalman-like smoother for single axis signals.
     * - x: filtered value
     * - p: estimate covariance (confidence)
     * - q: process noise (responsiveness)
     * - r: measurement noise (smoothing strength)
     */
    struct Smooth1D
    {
        float x = 0.0f;
        float p = 1.0f;
        float q = 0.001f;
        float r = 0.02f;
        bool init = false;
    };

    /**
     * Smoothing controls
     * - smoothEnable toggles smoothing usage in readAccel/readGyro.
     * - Each axis has its own filter state.
     */
    bool smoothEnable = true;

    Smooth1D sax;
    Smooth1D say;
    Smooth1D saz;

    Smooth1D sgx;
    Smooth1D sgy;
    Smooth1D sgz;

    /**
     * smoothUpdate
     * - Updates filter state with the new measurement and returns the filtered output.
     */
    float smoothUpdate(Smooth1D &s, float measurement);
};
