#include "LSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// to calibrate gyroscope
#define CALIBRATE_SIZE 100 // number of samples to calibrate with
#define COMPARE_SIZE 10

// calculated offsets from earlier runs,
// currently manually taken from Serial monitor output
// double gx_offset = 1.20;
// double gy_offset = -2.45;
// double gz_offset = 0.5;

double cal_gx = 0.0;
double cal_gy = 0.0;
double cal_gz = 0.0;

double gx_data = 0.0;
double gy_data = 0.0;
double gz_data = 0.0;

double gx_data_avg = 0.0;
double gy_data_avg = 0.0;
double gz_data_avg = 0.0;

double cal_ax = 0.0;
double cal_ay = 0.0;
double cal_az = 0.0;

double ax_data = 0.0;
double ay_data = 0.0;
double az_data = 0.0;

double ax_data_avg = 0.0;
double ay_data_avg = 0.0;
double az_data_avg = 0.0;

double zero_threshold = 0.01;
int num_samples = 0;
bool calibration_done = false;

void setup() {
    Serial.begin(9600);
    while (!Serial);
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}

void calibrate_gyro_offsets() {
    //Gyroscope
    Serial.print("\nGyroscope:\n");
    gx_data = myIMU.readFloatGyroX();
    gy_data = myIMU.readFloatGyroY();
    gz_data = myIMU.readFloatGyroZ();

    Serial.print(" X1 = ");
    cal_gx = (cal_gx * (CALIBRATE_SIZE - 1))/(CALIBRATE_SIZE) + (gx_data)/(CALIBRATE_SIZE);
    gx_data_avg = (gx_data_avg * (COMPARE_SIZE - 1))/(COMPARE_SIZE) + (gx_data)/(COMPARE_SIZE);
    Serial.println(cal_gx);
    Serial.print(" Y1 = ");
    cal_gy = (cal_gy * (CALIBRATE_SIZE - 1))/(CALIBRATE_SIZE) + (gy_data)/(CALIBRATE_SIZE);
    gy_data_avg = (gy_data_avg * (COMPARE_SIZE - 1))/(COMPARE_SIZE) + (gy_data)/(COMPARE_SIZE);
    Serial.println(cal_gy);
    Serial.print(" Z1 = ");
    cal_gz = (cal_gz * (CALIBRATE_SIZE - 1))/(CALIBRATE_SIZE) + (gz_data)/(CALIBRATE_SIZE);
    gz_data_avg = (gz_data_avg * (COMPARE_SIZE - 1))/(COMPARE_SIZE) + (gz_data)/(COMPARE_SIZE);
    Serial.println(cal_gz);

    //Thermometer
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C1 = ");
    Serial.println(myIMU.readTempC(), 4);
    Serial.print(" Degrees F1 = ");
    Serial.println(myIMU.readTempF(), 4);

    Serial.println("Current gyroscope offsets");
    Serial.print("gx offset: "); Serial.println(cal_gx);
    Serial.print("gy offset: "); Serial.println(cal_gy);
    Serial.print("gz offset: "); Serial.println(cal_gz);

}

void calibrate_accel_offsets() {
    //Accelerometer
    Serial.println("Accelerometer");
    ax_data = myIMU.readFloatAccelX();
    ay_data = myIMU.readFloatAccelY();
    az_data = myIMU.readFloatAccelZ();

    Serial.print(" AX1 = ");
    cal_ax = (cal_ax * (CALIBRATE_SIZE - 1))/(CALIBRATE_SIZE) + (ax_data)/(CALIBRATE_SIZE);
    ax_data_avg = (ax_data_avg * (COMPARE_SIZE - 1))/(COMPARE_SIZE) + (ax_data)/(COMPARE_SIZE);
    Serial.println(cal_ax);
    Serial.print(" AY1 = ");
    cal_ay = (cal_ay * (CALIBRATE_SIZE - 1))/(CALIBRATE_SIZE) + (ay_data)/(CALIBRATE_SIZE);
    ay_data_avg = (ay_data_avg * (COMPARE_SIZE - 1))/(COMPARE_SIZE) + (ay_data)/(COMPARE_SIZE);
    Serial.println(cal_ay);
    Serial.print(" Z1 = ");
    cal_az = (cal_az * (CALIBRATE_SIZE - 1))/(CALIBRATE_SIZE) + (az_data)/(CALIBRATE_SIZE);
    az_data_avg = (az_data_avg * (COMPARE_SIZE - 1))/(COMPARE_SIZE) + (az_data)/(COMPARE_SIZE);
    Serial.println(cal_az);

    Serial.println("Current accel offsets");
    Serial.print("ax offset: "); Serial.println(cal_ax);
    Serial.print("ay offset: "); Serial.println(cal_ay);
    Serial.print("az offset: "); Serial.println(cal_az);
}

void loop() {

    bool gyro_done = (abs(gx_data_avg - cal_gx) < zero_threshold) && (abs(gy_data_avg - cal_gy) < zero_threshold) && (abs(gz_data_avg - cal_gz) < zero_threshold);
    bool accel_done = (abs(ax_data_avg - cal_ax) < zero_threshold) && (abs(ay_data_avg - cal_ay) < zero_threshold) && (abs(az_data_avg - cal_az) < zero_threshold);

    if(!calibration_done) {
        if(!gyro_done || (num_samples < CALIBRATE_SIZE)) {
            calibrate_gyro_offsets();
        }
        if(!accel_done || (num_samples < CALIBRATE_SIZE)) {
            calibrate_accel_offsets();
        }
        Serial.println("Current offset errors");
        Serial.print("gx error: "); Serial.println(gx_data_avg - cal_gx);
        Serial.print("gy error: "); Serial.println(gy_data_avg - cal_gy);
        Serial.print("gz error: "); Serial.println(gz_data_avg - cal_gz);
        Serial.print("ax error: "); Serial.println(ax_data_avg - cal_ax);
        Serial.print("ay error: "); Serial.println(ay_data_avg - cal_ay);
        Serial.print("az error: "); Serial.println(az_data_avg - cal_az);
        num_samples = num_samples + 1;
    }

    if(!calibration_done && gyro_done && accel_done && (num_samples > CALIBRATE_SIZE)) {
        calibration_done = true;
        Serial.println("******Calibration is done**********");
        Serial.print("gx offset: "); Serial.println(cal_gx);
        Serial.print("gy offset: "); Serial.println(cal_gy);
        Serial.print("gz offset: "); Serial.println(cal_gz);
        Serial.print("ax offset: "); Serial.println(cal_ax);
        Serial.print("ay offset: "); Serial.println(cal_ay);
        Serial.print("az offset: "); Serial.println(cal_az);
        Serial.print("num samples taken: "); Serial.println(num_samples);
    }

    if(calibration_done) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    }

    delay(100);
}
