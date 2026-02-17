/**
 * ISM330DHCX FIFO Read Example
 * - Prints :
 *   - Accel in g
 *   - Gyro in dps
 *   - Temp in degC
 *
 * I2C wiring (typical):
 * - VCC -> 3.3V
 * - GND -> GND
 * - SDA -> SDA pin
 * - SCL -> SCL pin
 *
 * Address note:
 * - Most boards are 0x6A (SA0=0)
 * - Try 0x6B if not detected
 */

#include <7Semi_ISM330DHCX.h>

ISM330DHCX_7Semi imu;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  /**
   * - Initialize IMU bus layer
   * - 400 kHz is a good default for fast FIFO reads
   */
  if (!imu.begin(Wire, 0x6A, 400000))
  {
    Serial.println("ERROR: IMU begin() failed (WHO_AM_I/init)");
    while (1) { delay(100); }
  }

  /**
   * FIFO setup
   * - Watermark is optional for polling mode
   * - Mode 0x06 is library-specific in your .cpp
   */
  if (!imu.fifoSetWatermark(48))
  {
    Serial.println("WARN: fifoSetWatermark failed");
  }

  if (!imu.fifoEnable(0x06))
  {
    Serial.println("ERROR: fifoEnable failed");
    while (1) { delay(100); }
  }

  Serial.println("IMU FIFO Serial Monitor Ready");
}

void loop()
{
  /**
   * Drain FIFO
   * - Read multiple frames per loop so FIFO does not overflow
   * - Limit reads so loop does not block forever
   */
  const uint8_t maxFramesPerLoop = 16;

  for (uint8_t i = 0; i < maxFramesPerLoop; i++)
  {
    fifoSample s;

    /**
     * - fifoReadSample() returns false when FIFO is empty
     * - break so we don't keep looping with old data
     */
    if (!imu.fifoReadSample(s))
      break;

    /**
     * Print only the data that arrived
     * - This avoids spamming repeated values
     */
    if (s.newAccel)
    {
      s.newAccel = false;
      Serial.print("A[g] ");
      Serial.print(s.ax, 4); Serial.print(", ");
      Serial.print(s.ay, 4); Serial.print(", ");
      Serial.println(s.az, 4);
    }

    if (s.newGyro)
    {
      s.newGyro = false;
      Serial.print("G[dps] ");
      Serial.print(s.gx, 4); Serial.print(", ");
      Serial.print(s.gy, 4); Serial.print(", ");
      Serial.println(s.gz, 4);
    }

    if (s.newTemp)
    {
      s.newTemp = false;
      Serial.print("T[C] ");
      Serial.println(s.temp, 2);
    }
  }
}
