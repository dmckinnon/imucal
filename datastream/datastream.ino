// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

typedef union
{
  float value;
  byte bytes[4];
} floatunion_t;

void setup(void) {
  SerialUSB.begin(115200);
  while (!SerialUSB)
    delay(10); // will pause Zero until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
    int attempts = 0;
    while (1) {
      delay(10);
      attempts ++;
      if (attempts > 1000)
      {
        while(1)
        {
          SerialUSB.write("no");
        }
      }
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  delay(500);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  // TODO: confirm that we aren't getting dupes
  mpu.getEvent(&a, &g, &temp);

  // create bytestream of data:
  // 0-3: header bytes
  // 4-7: accel x
  // 8-11: accel y
  // 12-15: accel z
  // 16-19: gyro x
  // 20-23: gyro y
  // 24-27: gyro z
  byte bytestream[28];
  bytestream[0] = '0';
  bytestream[1] = '0';
  bytestream[2] = '0';
  bytestream[3] = '0';

  floatunion_t a_x, a_y, a_z, g_x, g_y, g_z;
  a_x.value = a.acceleration.x;
  a_y.value = a.acceleration.y;
  a_z.value = a.acceleration.z;
  g_x.value = g.gyro.x;
  g_y.value = g.gyro.y;
  g_z.value = g.gyro.z;

  for (int i = 0; i < 4; ++i)
  {
    bytestream[4+i] = a_x.bytes[i];
    bytestream[8+i] = a_y.bytes[i];
    bytestream[12+i] = a_z.bytes[i];
    bytestream[16+i] = g_x.bytes[i];
    bytestream[20+i] = g_y.bytes[i];
    bytestream[24+i] = g_z.bytes[i];
  }

  // Write packet header
  SerialUSB.write(bytestream, 28);

  // Should we delay or should we wait upon next packet, or just send whenevs?
  delay(0);
}
