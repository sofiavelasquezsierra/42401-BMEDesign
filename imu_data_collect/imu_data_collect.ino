#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>

// BLE UART service
BLEUart bleuart;

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

double cal_gx = 1.101100;
double cal_gy = -2.472750;
double cal_gz = 0.921550;

double cal_ax = -0.058177;
double cal_ay = -0.020211;
double cal_az = -0.001846;


void setup() {
    Serial.begin(115200);
    while (!Serial);
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

  // BLE init
  Bluefruit.begin();
  Bluefruit.setName("XIAO-IMU");

  bleuart.begin();

  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start(0); // continuous
}


void send_values() {
    float ax = myIMU.readFloatAccelX() - cal_ax;
    float ay = myIMU.readFloatAccelY() - cal_ay;
    float az = myIMU.readFloatAccelZ() - cal_az;

    float gx = myIMU.readFloatGyroX() - cal_gx;
    float gy = myIMU.readFloatGyroY() - cal_gy;
    float gz = myIMU.readFloatGyroZ() - cal_gz;

    char buffer[128];
    snprintf(buffer, sizeof(buffer),
            "%.3f,%.3f,%.3f,",
            ax, ay, az);
    bleuart.print(buffer);
    Serial.print(buffer);
    delay(50);

    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f,%.3f",
            gx, gy, gz);
    Serial.println(buffer);
    bleuart.print(buffer);
}

void loop() {

    send_values();

    delay(250);
}
