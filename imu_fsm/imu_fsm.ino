#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>


// constants
const float G = 9.81;

// macros
// These values are inspired by the paper
#define BUF_SIZE 250
#define IDLE_TRIGGER 0.8*G
#define CHECK_TRIGGER 1.4*G

// TODO: These values are most definitely incorrect, update
#define ACCEL_DEV_THRESHOLD 0.2
#define GYRO_DEV_THRESHOLD 0.2
#define TILT_THRESHOLD 0.3

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

float asvm_buf[BUF_SIZE];
float gsvm_buf[BUF_SIZE];

int update_pos;
bool avg_valid;

float A_SVM; // accelerometer signal vector magnitude
float A_SVM_mean;
float G_SVM;
float G_SVM_mean;

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
    STABILIZE_ACCEL_FALL = 2,
    STABILIZE_GYRO_FALL = 3,
    POSTURE_CHECK_FALL = 4,
    DETECTED_FALL = 5,
} fall_state;

enum IMU_COMP {
    ACCEL = 0,
    GYRO = 1
} imu_comp;

// enum LOCOMOTION_STATES {
//     STANDING = 0,
//     WALKING = 1,
//     RUNNING = 2
// } locomotion_state;

void initialize_values() {
    fall_state = IDLE_FALL;
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

    // update SVMs
    A_SVM = sqrt(sq(ax) + sq(ay) + sq(az));
    G_SVM = sqrt(sq(gx) + sq(gy) + sq(gz));

    // update buffers with new value
    ax_buf[update_pos] = ax;
    ay_buf[update_pos] = ay;
    az_buf[update_pos] = az;

    gx_buf[update_pos] = gx;
    gy_buf[update_pos] = gy;
    gz_buf[update_pos] = gz;

    // might not need
    asvm_buf[update_pos] = A_SVM;
    gsvm_buf[update_pos] = G_SVM;

    // increment position pointer
    update_pos = (update_pos + 1) % BUF_SIZE;

    // update means
    A_SVM_mean = (A_SVM_mean*(update_pos - 1))/(update_pos) + (A_SVM)/(update_pos);
    G_SVM_mean = (G_SVM_mean*(update_pos - 1))/(update_pos) + (G_SVM)/(update_pos);

    // update if the data is valid or not
    if(!avg_valid & (update_pos >= BUF_SIZE)) {
        avg_valid = true;
    }
    
}

// send all IMU data points over BLEs
void send_values() {
    float ax = myIMU.readFloatAccelX() - cal_ax;
    float ay = myIMU.readFloatAccelY() - cal_ay;
    float az = myIMU.readFloatAccelZ() - cal_az;

    float gx = myIMU.readFloatGyroX() - cal_gx;
    float gy = myIMU.readFloatGyroY() - cal_gy;
    float gz = myIMU.readFloatGyroZ() - cal_gz;

    A_SVM = sqrt(sq(ax) + sq(ay) + sq(az));
    G_SVM = sqrt(sq(gx) + sq(gy) + sq(gz));

    char buffer[128];
    // send accel values
    snprintf(buffer, sizeof(buffer),
            "%.3f,%.3f,%.3f,",
            ax, ay, az);
    bleuart.print(buffer);
    Serial.print(buffer);
    delay(50);

    // send gyro values
    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f,%.3f",
            gx, gy, gz);
    Serial.print(buffer);
    bleuart.print(buffer);
    delay(50);

    // send svm values
    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f",
            A_SVM, G_SVM);
    Serial.println(buffer);
    bleuart.print(buffer);
    delay(50);
}

// returns if there was a high acceleration event, also collect the BUF_SIZE
// samples for later processing
bool check_fall() {
    bool large_accel = false;
    // reset update position
    update_pos = 0;
    for(int i = 0; i < BUF_SIZE; i++) {
        update_values();
        if(A_SVM >= CHECK_TRIGGER) {
            large_accel = true;
        }
    }
    return large_accel;
}

bool std_dev_check(IMU_COMP dev_type, float threshold) {
    float sum = 0.0;
    float std = 0.0;
    // naive implementation, optimize if lag is too bad
    if(dev_type == ACCEL) {
        for(int i = 0; i < BUF_SIZE; i++) {
        sum = ((sum + asvm_buf[i])*(sum + asvm_buf[i]))/(BUF_SIZE);
        }
    }
    else {
        for(int i = 0; i < BUF_SIZE; i++) {
        sum = ((sum + gsvm_buf[i])*(sum + gsvm_buf[i]))/(BUF_SIZE);
        }
    }

    std = sqrt(sum);

    return (std >= threshold);
    
}

bool posture_check() {
    return true;
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

// FSM style motion detection
void loop() {
    send_values();

    // while(fall_state == IDLE_FALL) {
    //     update_values();
    //     // right now this is just a instantaneous check but really should 
    //     // be some small running average
    //     if(A_SVM >= IDLE_TRIGGER) {
    //         fall_state = CHECK_FALL;
    //     }
    // }

    // if(fall_state == CHECK_FALL) {
    //     if(check_fall()) {
    //         fall_state = STABILIZE_ACCEL_FALL;
    //     }
    //     else {
    //         fall_state = IDLE_FALL;
    //     }
    // }

    // if(fall_state == STABILIZE_ACCEL_FALL) {
    //     if(std_dev_check(ACCEL, ACCEL_DEV_THRESHOLD)) {
    //         fall_state = STABILIZE_GYRO_FALL;
    //     }

    //     else {
    //         fall_state = IDLE_FALL;
    //     }
    // }

    // if(fall_state == STABILIZE_GYRO_FALL) {
    //     if(std_dev_check(GYRO, GYRO_DEV_THRESHOLD)) {
    //         fall_state = POSTURE_CHECK_FALL;
    //     }

    //     else {
    //         fall_state = IDLE_FALL;
    //     }
    // }

    // if(fall_state == POSTURE_CHECK_FALL) {
    //     if(posture_check(TILT_TRIGGER)) {
    //         fall_state = DETECTED_FALL;
    //     }
    //     else {
    //         fall_state = IDLE_FALL;
    //     }
    // }

    // if(fall_state == DETECTED_FALL) {
    //     send_values();
    // }

    delay(100);
}
