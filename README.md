# 7Semi ISM330DHCX Arduino Library

Arduino library for the **STMicroelectronics ISM330DHCX** 6-axis IMU  
(3-axis accelerometer + 3-axis gyroscope).
This library provides a clean and lightweight interface to configure and read data from the ISM330DHCX over **I2C**.

---

##  Features

- I2C communication interface
- 3-axis accelerometer readings (X, Y, Z)
- 3-axis gyroscope readings (X, Y, Z)
- Configurable Output Data Rate (ODR)
- Configurable full-scale range
- FIFO support

---

##  Hardware Requirements

- ISM330DHCX IMU sensor
- Arduino-compatible board  
  (UNO, Nano, Mega, ESP32, etc.)
- I2C connection

---

## 🔌 I2C Wiring Example (Arduino UNO)

| ISM330DHCX | Arduino UNO |
|------------|------------|
| VCC        | 3.3V       |
| GND        | GND        |
| SDA        | A4         |
| SCL        | A5         |

> ⚠️ The ISM330DHCX operates at 3.3V logic. Use level shifting if required.

---

##  Installation

### Method 1 — Manual Installation

1. Download this repository as ZIP.
2. Extract the folder.
3. Move it to your Arduino libraries folder
4. Restart the Arduino IDE.

---

##  Examples

After installation:

File → Examples → 7Semi_ISM330DHCX

vailable example code:

- Accelerometer reading
- Gyroscope reading
- FIFO usage
- Configuration examples

---
## Library Function Reference

### Begin

- `begin(TwoWire &wire, uint8_t addr=0x6A, uint32_t clockHz=400000)`  
  Verifies the sensor ID and applies a safe default configuration for operation.

- `getProductID(uint8_t &id)`  
  Reads the sensor “Who-Am-I” / product ID into `id`.

---

### Sensor Reads

- `readAccel(float &x, float &y, float &z)`  
  Reads accelerometer axes in **g** units (scaled).

- `readAccelMS2(float &x, float &y, float &z)`  
  Reads accelerometer axes in **m/s²** units.

- `readGyro(float &x, float &y, float &z)`  
  Reads gyroscope axes in **dps** (degrees/sec).

- `readGyroRads(float &x, float &y, float &z)`  
  Reads gyroscope axes in **rad/s** units.

- `readTemp(float &t)`  
  Reads temperature in **°C**.

---

### FIFO Control

- `fifoEnable(uint8_t mode=0x06)`  
  Enables FIFO with the provided mode value (device register field value).

- `fifoDisable()`  
  Disables FIFO and returns sensor streaming to normal mode.

- `fifoSetWatermark(uint16_t wtm)`  
  Sets FIFO watermark threshold for interrupts/ready checks.

- `fifoRead(uint8_t *buf, uint16_t len)`  
  Reads raw FIFO bytes into a user buffer.

- `fifoReady()`  
  Returns true when FIFO watermark/ready condition indicates data is available.

- `fifoReadSample(fifoSample &sample)`  
  Reads one decoded FIFO sample into a structured `fifoSample`.

- `parseFifoTaggedFrames(const uint8_t *buf, size_t bytes, fifoSample *outSamples, size_t maxSamples)`  
  Parses tagged FIFO frames from a byte buffer into a `fifoSample` array.

---

### Interrupt Pin Routing / Status

- `interruptPolarity(bool activeHigh)`  
  Sets interrupt polarity active-high or active-low.

- `setInterruptOutputMode(bool openDrain)`  
  Selects push-pull or open-drain interrupt output mode.

- `routeAllInterruptsToINT1(bool en)`  
  Routes interrupt signals to INT1 when enabled (device routing helper).

- `clearInterrupts()`  
  Clears latched interrupt sources by reading/clearing related status.

---

### Configuration (Accel / Gyro)

- `setAccelConfig(ISM330_XL_ODR odr, ISM330_XL_FS fs, bool lpf2)`  
  Sets accelerometer ODR, full-scale range, and LPF2 enable.

- `setGyroConfig(ISM330_G_ODR odr, ISM330_G_FS fs, bool lpf1)`  
  Sets gyroscope ODR, full-scale range, and LPF1 enable.

- `setAccelODR(ISM330_XL_ODR odr)`  
  Updates only accelerometer output data rate.

- `setAccelFS(ISM330_XL_FS fs)`  
  Updates only accelerometer full-scale range.

- `setAccelLPF2(bool enable)`  
  Enables or disables accelerometer LPF2 path.

- `setGyroODR(ISM330_G_ODR odr)`  
  Updates only gyroscope output data rate.

- `setGyroFS(ISM330_G_FS fs)`  
  Updates only gyroscope full-scale range.

- `setGyroLPF1(ISM330_G_LPF1 bw)`  
  Selects gyro LPF1 bandwidth option.

- `enableGyroLPF1(bool en)`  
  Enables/disables gyro LPF1 in the device chain.

- `setAccelFilter(uint8_t hpcf, bool highPass)`  
  Configures accelerometer filter cutoff and selects high-pass or low-pass behavior.

- `setGyroLPF1Bandwidth(uint8_t ftype)`  
  Sets gyro filter type/bandwidth using raw field value (advanced use).

---

### Data Integrity / Register Helpers

- `enableBDU(bool en)`  
  Enables block-data-update to avoid partial reads during updates.

- `setAutoIncrement(bool en)`  
  Enables register auto-increment for multi-byte burst reads/writes.

- `getNewDataStatus(bool &a, bool &g, bool &t)`  
  Reports whether new accel/gyro/temp data is ready.

- `enableDrdyMask(bool en)`  
  Masks/unmasks data-ready behavior as supported by the device.

---

### Reset

- `softwareReset()`  
  Performs a software reset and returns sensor registers to defaults.

---

### Power / Performance Modes

- `setAccelHighPerformance(bool enable)`  
  Switches accelerometer between high-performance and lower-power mode.

- `setGyroHighPerf(bool enable)`  
  Switches gyroscope between high-performance and lower-power mode.

- `enableGyroSleep(bool en)`  
  Enables/disables gyro sleep behavior for power saving.

---
