/**
 * Example: Configure ODR/FS 
 * - Sets accel + gyro ODR/FS after begin().
 * - Routes accel DRDY and gyro DRDY
 *
 * Serial output:
 * - Prints accel in g and gyro in dps whenever DRDY triggers.
 *
 * I2C wiring (typical):
 * - VCC -> 3.3V
 * - GND -> GND
 * - SDA -> SDA pin
 * - SCL -> SCL pin
 */

#include <7Semi_ISM330DHCX.h>

ISM330DHCX_7Semi imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  /**
   * - Initialize IMU over I2C
   * - Default address 0x6A; use 0x6B if your board straps SA0=1
   */
  if (!imu.beginI2C(Wire, 0x6A, 400000)) {
    Serial.println("ERROR: IMU begin() failed (WHO_AM_I/init)");
    while (1) { delay(100); }
  }

  /**
   * Sensor config example:
   * - Accel: 104 Hz, ±4g, LPF2 enabled
   * - Gyro : 104 Hz, 500 dps, LPF1 enabled
   */
  imu.setAccelConfig(XL_ODR_104Hz, XL_FS_4G, true);
  imu.setGyroConfig(G_ODR_104Hz, G_FS_500, true);

  Serial.println("Configured ODR/FS DRDY enabled");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  /**
   * - Read accel and gyro
   * - Reads should clear DRDY status internally when registers are read
   */
  if (imu.readAccel(ax, ay, az) && imu.readGyro(gx, gy, gz)) {
    Serial.print("A[g] ");
    Serial.print(ax, 4);
    Serial.print(", ");
    Serial.print(ay, 4);
    Serial.print(", ");
    Serial.print(az, 4);

    Serial.print(" | G[dps] ");
    Serial.print(gx, 3);
    Serial.print(", ");
    Serial.print(gy, 3);
    Serial.print(", ");
    Serial.println(gz, 3);
  } else {
    Serial.println("WARN: readAccel/readGyro failed");
  }
}
