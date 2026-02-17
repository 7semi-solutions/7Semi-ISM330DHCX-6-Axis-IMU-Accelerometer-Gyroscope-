/**
 * ISM330DHCX FIFO Serial Plotter Example
 * Plots:
 * gx gy gz ax ay az temp
 *
 * Open: Tools → Serial Plotter
 * Baud: 115200
 */

#include <Arduino.h>
#include <Wire.h>
#include "7Semi_ISM330DHCX.h"

ISM330DHCX_7Semi imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();

  if (!imu.beginI2C(Wire, 0x6A, 400000)) {
    Serial.println("I2C init failed");
    while (1) { delay(100); }
  }

  if (!imu.begin()) {
    Serial.println("IMU begin failed");
    while (1) { delay(100); }
  }

  // Optional timestamp
  // imu.enableTimestamp(true);

  // FIFO setup
  imu.fifoSetWatermark(48);
  imu.fifoEnable(0x06);  // Enable accel + gyro to FIFO

  Serial.println("IMU FIFO Plotter Ready");
}

void loop() {
  fifoSample s;

  // Drain multiple FIFO frames each loop
  if (!imu.fifoReadSample(s));

  // Only print when new data arrives
  if (s.newGyro || s.newAccel || s.newTemp) {
    // Print as numeric columns (IMPORTANT for plotter)
    Serial.print(s.gx);
    Serial.print(" ");
    Serial.print(s.gy);
    Serial.print(" ");
    Serial.print(s.gz);
    Serial.print(" ");

    Serial.print(s.ax);
    Serial.print(" ");
    Serial.print(s.ay);
    Serial.print(" ");
    Serial.print(s.az);
    Serial.print(" ");

    Serial.println(s.temp);  // last value must use println
  }

  // delay(5);  // small delay for stability
}
