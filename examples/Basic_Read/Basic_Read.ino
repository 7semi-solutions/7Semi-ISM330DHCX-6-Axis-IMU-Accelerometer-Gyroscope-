#include <7Semi_ISM330DHCX.h>

/** - Create IMU driver object */
ISM330DHCX_7Semi imu;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  /**
   * I2C CONNECTION (Typical Arduino)
   * - VCC  -> 3.3V (recommended for ISM330DHCX boards)
   * - GND  -> GND
   * - SDA  -> SDA pin (UNO: A4, Mega: 20, ESP32: GPIO21 default)
   * - SCL  -> SCL pin (UNO: A5, Mega: 21, ESP32: GPIO22 default)
   *
   * Address note:
   * - Most ISM330DHCX boards use 0x6A when SA0 = 0
   * - If your board is wired SA0 = 1, address becomes 0x6B
   * - begin() does:
   * - Start I2C bus for the IMU.
   * - clockHz = 400k is a good default for fast reads.
   * - WHO_AM_I check
   * - basic setup (BDU, IF_INC, and defaults)
   */
  if (!imu.begin(Wire, 0x6A, 400000))
  {
    Serial.println("ERROR: IMU begin() failed (WHO_AM_I/init)");
    while (1) { delay(100); }
  }

  /**
   * Optional config
   * - Set known rates/ranges so scaling is predictable.
   * - You can remove this if begin() already sets your preferred defaults.
   */
  imu.setAccelConfig(XL_ODR_104Hz, XL_FS_4G, true);
  imu.setGyroConfig(G_ODR_104Hz, G_FS_500, true);

  Serial.println("I2C Basic Read started");
}

void loop()
{
  float ax, ay, az;
  float gx, gy, gz;
  float t;

  /**
   * - Read sensor values.
   * - All functions return false on bus/register failure.
   */
  bool okA = imu.readAccel(ax, ay, az);
  bool okG = imu.readGyro(gx, gy, gz);
  bool okT = imu.readTemp(t);

  if (okA && okG && okT)
  {
    Serial.print("A[g] ");
    Serial.print(ax, 4); Serial.print(", ");
    Serial.print(ay, 4); Serial.print(", ");
    Serial.print(az, 4);

    Serial.print(" | G[dps] ");
    Serial.print(gx, 4); Serial.print(", ");
    Serial.print(gy, 4); Serial.print(", ");
    Serial.print(gz, 4);

    Serial.print(" | T[C] ");
    Serial.println(t, 2);
  }
  else
  {
    Serial.println("WARN: read failed (check I2C stability)");
  }

  delay(50);
}
