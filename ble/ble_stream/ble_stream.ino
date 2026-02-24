#include <Wire.h>
#include <MAX30105.h>

#include <Arduino.h>
#include <LSM6DS3.h>
#include <bluefruit.h>

BLEUart bleuart;

// Sensors
MAX30105 sensor;
LSM6DS3 imu(I2C_MODE, 0x6A);

// PPG timing
uint32_t last_ts = 0;
uint32_t t0 = 0;
const uint32_t dt = 10000; // 100 Hz -> 10,000 us
uint32_t counter = 0;

// BLE send decimation: 100 Hz samples -> send 1/4 => 25 Hz
uint8_t ble_decim = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  t0 = micros();

  // IMU init
  if (imu.begin() != 0) {
    Serial.println("IMU not detected!");
    while (1) {}
  }

  // BLE init
  Bluefruit.begin();
  Bluefruit.setName("XIAO-SENSE");
  bleuart.begin();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start(0);

  // PPG init
  if (!sensor.begin(Wire, 400000)) {
    Serial.println("MAX30102 not found");
    while (1) {}
  }

  sensor.setup(
    60,   // LED power
    4,    // sample average
    2,    // red + IR
    100,  // 100 Hz
    411,  // pulse width
    4096  // ADC range
  );

  Serial.println("PPG+IMU ready.");
}

void loop() {
  int n = sensor.check();

  if (n == 0) {
    last_ts = micros();
    counter = 0;
    return;
  }

  if (counter == 0) {
    last_ts = micros();
  }

  while (n--) {
    uint32_t ir_raw  = sensor.getFIFOIR();
    uint32_t red_raw = sensor.getFIFORed();

    if (counter > 0) {
      last_ts += dt;
    }

    // Serial Plotter: IR,RED
    // Serial.print(ir_raw);
    // Serial.print(",");
    // Serial.println(red_raw);

    // BLE send
    ble_decim++;
    if ((ble_decim >= 4) | Serial) {
      ble_decim = 0;

      // stream data either when BLE connected or serial connected
      if (Bluefruit.connected() | Serial) {
        // Read IMU once per BLE tick
        float ax = imu.readFloatAccelX();
        float ay = imu.readFloatAccelY();
        float az = imu.readFloatAccelZ();
        float gx = imu.readFloatGyroX();
        float gy = imu.readFloatGyroY();
        float gz = imu.readFloatGyroZ();

        float ASVM = sqrt(ax*ax + ay*ay + az*az);
        float GSVM = sqrt(gx*gx + gy*gy + gz*gz);

        Serial.print("IR_RAW: "); Serial.print(ir_raw); Serial.print("; ");
        Serial.print("RED_RAW: "); Serial.print(red_raw); Serial.print("; ");

        Serial.print("AX: "); Serial.print(ax); Serial.print("; ");
        Serial.print("AY: "); Serial.print(ay); Serial.print("; ");
        Serial.print("AZ: "); Serial.print(az); Serial.print("; ");

        Serial.print("GX: "); Serial.print(gx); Serial.print("; ");
        Serial.print("GY: "); Serial.print(gy); Serial.print("; ");
        Serial.print("GZ: "); Serial.print(gz); Serial.print("; ");

        Serial.print("ASVM: "); Serial.print(ASVM); Serial.print("; ");
        Serial.print("GSVM: "); Serial.print(GSVM); Serial.print("; ");


        // Scale to fixed-point
        // accel: 0.01 g
        int32_t ax_i = (int32_t)lroundf(ax * 100.0f);
        int32_t ay_i = (int32_t)lroundf(ay * 100.0f);
        int32_t az_i = (int32_t)lroundf(az * 100.0f);

        // gyro: 0.1 dps
        int32_t gx_i = (int32_t)lroundf(gx * 10.0f);
        int32_t gy_i = (int32_t)lroundf(gy * 10.0f);
        int32_t gz_i = (int32_t)lroundf(gz * 10.0f);

        if (ax_i > 32767 || ax_i < -32768 ||
            ay_i > 32767 || ay_i < -32768 ||
            az_i > 32767 || az_i < -32768 ||
            gx_i > 32767 || gx_i < -32768 ||
            gy_i > 32767 || gy_i < -32768 ||
            gz_i > 32767 || gz_i < -32768) {
          Serial.println("WARNING: IMU fixed-point overflow (int16). Increase scaling or change format.");
        }

        // Cast to int16 for transport
        int16_t ax16 = (int16_t)ax_i, ay16 = (int16_t)ay_i, az16 = (int16_t)az_i;
        int16_t gx16 = (int16_t)gx_i, gy16 = (int16_t)gy_i, gz16 = (int16_t)gz_i;

        // Keep 18-bit PPG samples
        ir_raw  &= 0x3FFFF;
        red_raw &= 0x3FFFF;

        uint32_t ts = last_ts - t0;
        Serial.print("MCU_TIME: "); Serial.println(ts);

        // P packet: 11 bytes
        // 'P' + ts(4) + ir(3) + red(3)
        uint8_t p[11];
        p[0] = 'P';
        memcpy(&p[1], &ts, 4);

        // IR 18-bit -> 3 bytes little-endian
        p[5] = (uint8_t)(ir_raw & 0xFF);
        p[6] = (uint8_t)((ir_raw >> 8) & 0xFF);
        p[7] = (uint8_t)((ir_raw >> 16) & 0x03);

        // RED 18-bit -> 3 bytes little-endian
        p[8]  = (uint8_t)(red_raw & 0xFF);
        p[9]  = (uint8_t)((red_raw >> 8) & 0xFF);
        p[10] = (uint8_t)((red_raw >> 16) & 0x03);
  
        // A packet: 11 bytes
        uint8_t a[11];
        a[0] = 'A';
        memcpy(&a[1], &ts, 4);
        memcpy(&a[5], &ax16, 2);
        memcpy(&a[7], &ay16, 2);
        memcpy(&a[9], &az16, 2);


        // G packet: 11 bytes
        uint8_t g[11];
        g[0] = 'G';
        memcpy(&g[1], &ts, 4);
        memcpy(&g[5], &gx16, 2);
        memcpy(&g[7], &gy16, 2);
        memcpy(&g[9], &gz16, 2);

        if(Bluefruit.connected()) {
          bleuart.write(p, sizeof(p));
          bleuart.write(a, sizeof(a));
          bleuart.write(g, sizeof(g));
        }
      }

      // boost sampling rate if just going over Serial port
      if(!Bluefruit.connected() & Serial) {
        delay(25);
      }
    }

    counter++;
  }
}