#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>

// constants
#define BUF_SIZE 10

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

// buffers for holding window of data
float gx_buf[BUF_SIZE];
float gy_buf[BUF_SIZE];
float gz_buf[BUF_SIZE];

float ax_buf[BUF_SIZE];
float ay_buf[BUF_SIZE];
float az_buf[BUF_SIZE];

int update_pos;
bool avg_valid;

float A_SVM; // accelerometer signal vector magnitude
float G_SVM;

// motion classifer states 
enum MOTION_STATES {
    IDLE = 0,
    WALK = 1,
    RUN = 2,
    LYING = 3,
    FALLEN = 4,
    STANDING = 5
} state;

enum FALL_STATES {
    IDLE_FALL = 0,
    CHECK_FALL = 1,
    IMMOBILE_FALL_FORWARD = 2,
    IMMOBILE_FALL_BACKWARD = 3,
    IMMOBILE_FALL_LEFT = 4,
    IMMOBILE_FALL_RIGHT = 5,
} fall_state;

// enum LOCOMOTION_STATES {
//     STANDING = 0,
//     WALKING = 1,
//     RUNNING = 2
// } locomotion_state;

void initialize_values() {
    state = IDLE;
    A_SVM = 0.0;
    G_SVM = 0.0;
    update_pos = 0;
    avg_valid = false;
}

void update_values() {
    // read from the IMU
    float ax = myIMU.readFloatAccelX() - cal_ax;
    float ay = myIMU.readFloatAccelY() - cal_ay;
    float az = myIMU.readFloatAccelZ() - cal_az;

    float gx = myIMU.readFloatGyroX() - cal_gx;
    float gy = myIMU.readFloatGyroY() - cal_gy;
    float gz = myIMU.readFloatGyroZ() - cal_gz;

    A_SVM = sqrt(sq(ax) + sq(ay) + sq(az));
    G_SVM = sqrt(sq(gx) + sq(gy) + sq(gz));

    ax_buf[update_pos] = ax;
    ay_buf[update_pos] = ay;
    az_buf[update_pos] = az;

    gx_buf[update_pos] = gx;
    gy_buf[update_pos] = gy;
    gz_buf[update_pos] = gz;

    update_pos = update_pos + 1;

    if(!avg_valid & (update_pos >= BUF_SIZE)) {
        avg_valid = true;
    }
    
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

    initialize_values();
}

void loop() {

    send_values();

    delay(250);
}
