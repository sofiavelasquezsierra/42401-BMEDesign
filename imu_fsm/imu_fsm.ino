#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>


// constants
const float G = 9.81;
const float RAD_TO_DEG_CONV = 57.295779;

// macros

#define LOOP_DELAY 25
// These values are inspired by the paper
#define BUF_SIZE 200
#define IDLE_TRIGGER 0.75 // 0.8 in paper but that was slightly too lenient
#define CHECK_TRIGGER 1.4

// These values are inspired by the data
#define ACCEL_DEV_THRESHOLD 0.06 // 0.1 in the paper
#define GYRO_DEV_THRESHOLD 16.3 // 10 in the paper
#define DEV_BUFFER_SIZE 50 // same as paper

// straight comparison to 60 degrees in the paper
#define TILT_TRIGGER_ANGLE 60
// relative comparison
#define TILT_TRIGGER 0.3
#define INIT_TILT_SIZE 0.1*BUF_SIZE
#define FINAL_TILT_SIZE 0.4*BUF_SIZE
#define STATIONARY_THRESHOLD 0.15

// Note: the AY value is actually the "AZ" due to the orientation of the device lol

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

float A_SVM_mean;
float G_SVM_mean;

struct curr_vals_struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float A_SVM; // signal vector magnitude
    float G_SVM;
    uint32_t curr_time;
    uint32_t delta_time;
    float fall_event_val;
};

curr_vals_struct cv = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};

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
    STATIONARY_POST_FALL = 6
} fall_state;

String fall_state_strings[6] = {"IDLE_FALL", "CHECK_FALL", "STABILIZE_ACCEL_FALL", "STABILIZE_GYRO_FALL", "POSTURE_CHECK_FALL", "DETECTED_FALL"};

enum IMU_COMP {
    ACCEL = 0,
    GYRO = 1
} imu_comp;


void initialize_values() {
    fall_state = IDLE_FALL;
    cv.A_SVM = 0.0;
    cv.G_SVM = 0.0;
    update_pos = 0;
    avg_valid = false;
}

void update_values(bool update_buffers) {
    // read from the IMU
    cv.ax = myIMU.readFloatAccelX() - cal_ax;
    cv.ay = myIMU.readFloatAccelY() - cal_ay;
    cv.az = myIMU.readFloatAccelZ() - cal_az;

    cv.gx = myIMU.readFloatGyroX() - cal_gx;
    cv.gy = myIMU.readFloatGyroY() - cal_gy;
    cv.gz = myIMU.readFloatGyroZ() - cal_gz;

    // update SVMs
    cv.A_SVM = sqrt(sq(cv.ax) + sq(cv.ay) + sq(cv.az));
    cv.G_SVM = sqrt(sq(cv.gx) + sq(cv.gy) + sq(cv.gz));

    // update buffers with new value
    if(update_buffers) {
        ax_buf[update_pos] = cv.ax;
        ay_buf[update_pos] = cv.ay;
        az_buf[update_pos] = cv.az;

        gx_buf[update_pos] = cv.gx;
        gy_buf[update_pos] = cv.gy;
        gz_buf[update_pos] = cv.gz;

        // might not need
        asvm_buf[update_pos] = cv.A_SVM;
        gsvm_buf[update_pos] = cv.G_SVM;

        // increment position pointer
        update_pos = (update_pos + 1) % BUF_SIZE;

        // update means
        A_SVM_mean = (A_SVM_mean*(update_pos - 1))/(update_pos) + (cv.A_SVM)/(update_pos);
        G_SVM_mean = (G_SVM_mean*(update_pos - 1))/(update_pos) + (cv.G_SVM)/(update_pos);

        // update if the data is valid or not
        if(!avg_valid & (update_pos >= BUF_SIZE)) {
            avg_valid = true;
        }
    }
    
}

void print_values() {
    Serial.print("AX: "); Serial.print(cv.ax); Serial.print(", ");
    Serial.print("AY: "); Serial.print(cv.ay); Serial.print(", ");
    Serial.print("AZ: "); Serial.print(cv.az); Serial.print(", ");

    Serial.print("GX: "); Serial.print(cv.gx); Serial.print(", ");
    Serial.print("GY: "); Serial.print(cv.gy); Serial.print(", ");
    Serial.print("GZ: "); Serial.print(cv.gz); Serial.print(", ");

    Serial.print("ASVM: "); Serial.print(cv.A_SVM); Serial.print(", ");
    Serial.print("GSVM: "); Serial.print(cv.G_SVM); Serial.print(", ");

    Serial.print("FALL_EVENT: "); Serial.print(cv.fall_event_val); Serial.print(", ");
    Serial.print("STATE: "); Serial.println(fall_state);
}

// send all IMU data points over BLEs
void send_values_BLE() {

    char buffer[160];

    cv.delta_time = millis() - cv.curr_time; // very naive, optimize later
    cv.curr_time = cv.curr_time + cv.delta_time;

    // send accel values
    snprintf(buffer, sizeof(buffer),
            "%.3f,%.3f,%.3f",
            cv.ax, cv.ay, cv.az);
    bleuart.print(buffer);
    Serial.print(buffer);
    delay(10);

    // send gyro values
    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f,%.3f",
            cv.gx, cv.gy, cv.gz);
    Serial.print(buffer);
    bleuart.print(buffer);
    delay(10);

    // send svm values
    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f",
            cv.A_SVM, cv.G_SVM);
    Serial.print(buffer);
    bleuart.print(buffer);
    delay(10);

      // send time and fall_event values
    snprintf(buffer, sizeof(buffer),
            ",%.lu,%.3f",
            cv.delta_time, cv.fall_event_val);
    Serial.println(buffer);
    bleuart.print(buffer);
    delay(10);

    delay(25);
}

void send_values_serial() {

    update_values(1);
    char buffer[160];

    cv.delta_time = millis() - cv.curr_time; // very naive, optimize later
    cv.curr_time = cv.curr_time + cv.delta_time;

    // send accel values
    snprintf(buffer, sizeof(buffer),
            "%.3f,%.3f,%.3f",
            cv.ax, cv.ay, cv.az);
    Serial.print(buffer);

    // send gyro values
    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f,%.3f",
            cv.gx, cv.gy, cv.gz);
    Serial.print(buffer);

    // send svm values
    snprintf(buffer, sizeof(buffer),
            ",%.3f,%.3f",
            cv.A_SVM, cv.G_SVM);
    Serial.print(buffer);
      // send time and fall_event values
    snprintf(buffer, sizeof(buffer),
            ",%.lu,%.3f,",
            cv.delta_time, cv.fall_event_val);
    Serial.print(buffer);
    Serial.println(fall_state_strings[fall_state]);
}

// void send_values() {

//     char buffer[160];

//     cv.curr_time = millis();

//     snprintf(buffer, sizeof(buffer),
//         "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
//         cv.curr_time,
//         cv.fall_event_val,
//         cv.ax, cv.ay, cv.az,
//         cv.gx, cv.gy, cv.gz,
//         cv.A_SVM, cv.G_SVM
//     );

//     bleuart.println(buffer);   // BLE UART
//     Serial.println(buffer);    // USB serial (optional)
// }


// returns if there was a high acceleration event, also collect the BUF_SIZE
// samples for later processing
bool check_fall() {
    bool large_accel = false;
    // reset update position
    update_pos = 0;
    for(int i = 0; i < BUF_SIZE; i++) {
        update_values(1);
        if(cv.A_SVM >= CHECK_TRIGGER) {
            large_accel = true;
        }
    }
    return large_accel;
}

// Calculate stdev over the last DEV_BUFFER_SIZE samples of the buffer
// If doesn't work, consider doing a comparison between earlier samples
// and later samples to show stabilization
bool std_dev_check(IMU_COMP dev_type, float threshold) {
    float sum = 0.0;
    float std = 0.0;
    // naive implementation, optimize if lag is too bad
    if(dev_type == ACCEL) {
        for(int i = BUF_SIZE - DEV_BUFFER_SIZE; i < BUF_SIZE; i++) {
        sum = ((sum + asvm_buf[i])*(sum + asvm_buf[i]))/(BUF_SIZE);
        }
    }
    else {
        for(int i = BUF_SIZE - DEV_BUFFER_SIZE ; i < BUF_SIZE; i++) {
        sum = ((sum + gsvm_buf[i])*(sum + gsvm_buf[i]))/(BUF_SIZE);
        }
    }

    std = sqrt(sum);

    return (std >= threshold);
    
}

// really just a angle percent difference check
bool posture_check() {
    float tilt_diff = 0.0;
    float tilt_init_sum = 0.0;
    float tilt_final_sum = 0.0;
    float hor_dist = 0.0;

    // extremely naive implementation, optimize later
    for(int i = 0; i < INIT_TILT_SIZE; i++) {
        hor_dist = sqrt(ax_buf[i]*ax_buf[i], az_buf[i]*az_buf[i]);
        tilt_init_sum += atan2(ay_buf[i], hor_dist);
    }
    for(int i = BUF_SIZE - FINAL_TILT_SIZE; i < BUF_SIZE; i++) {
        hor_dist = sqrt(ax_buf[i]*ax_buf[i], az_buf[i]*az_buf[i]);
        tilt_final_sum += atan2(ay_buf[i], hor_dist);
    }

    tilt_diff = (abs((tilt_final_sum/FINAL_TILT_SIZE) - (tilt_init_sum/INIT_TILT_SIZE)))/(tilt_init_sum);
    Serial.print("Calculated angle: "); Serial.print(tilt_diff); Serial.print(", TILT_TRIGGER: "); Serial.println(TILT_TRIGGER);
    return (tilt_diff >= TILT_TRIGGER);
}

bool posture_check_angle() {
    float tilt_final_sum = 0.0;
    float hor_dist = 0.0;

    // angle to the horizontal plane among last samples
    for(int i = BUF_SIZE - FINAL_TILT_SIZE; i < BUF_SIZE; i++) {
        hor_dist = sqrt(ax_buf[i]*ax_buf[i], az_buf[i]*az_buf[i]);
        tilt_final_sum += atan2(ay_buf[i], hor_dist);
    }

    float tilt_final_angle = abs(((tilt_final_sum*RAD_TO_DEG_CONV)/FINAL_TILT_SIZE))
    Serial.print("Calculated angle: "); Serial.print(tilt_final_angle); Serial.print(", TILT_TRIGGER: "); Serial.println(TILT_TRIGGER_ANGLE);
    return (tilt_final_angle <= TILT_TRIGGER_ANGLE);
}

void update_buffer() {
    // reset update position
    update_pos = 0;
    for(int i = 0; i < BUF_SIZE; i++) {
        update_values(1);
    }
    return;
}

bool check_stationary() {
    bool stationary = false;
    float sum = 0.0;
    for(int i = 0; i < BUF_SIZE; i++) {
        sum = sum + asvm_buf[i];
    }

    float avg_asvm = sum / BUF_SIZE;
    // ASVM = 1 when stationary
    stationary = (abs(avg_asvm - 1) <= STATIONARY_THRESHOLD);
    return stationary;
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
    // send_values();

    if(fall_state == IDLE_FALL) {
        // Serial.println("IN IDLE_FALL");
        // update_values(1);
        send_values_serial();
        // right now this is just a instantaneous check but really should 
        // be some small running average
        if(cv.A_SVM <= IDLE_TRIGGER) {
            fall_state = CHECK_FALL;
        }
        else {
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == CHECK_FALL) {
        send_values_serial();
        Serial.println("IN CHECK_FALL");
        if(check_fall()) {
            fall_state = STABILIZE_ACCEL_FALL;
        }
        else {
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == STABILIZE_ACCEL_FALL) {
        send_values_serial();
        Serial.println("IN STABLIZE_ACCEL_FALL");
        if(std_dev_check(ACCEL, ACCEL_DEV_THRESHOLD)) {
            fall_state = STABILIZE_GYRO_FALL;
        }

        else {
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == STABILIZE_GYRO_FALL) {
        send_values_serial();
        Serial.println("IN STABILIZE_GYRO_FALL");
        if(std_dev_check(GYRO, GYRO_DEV_THRESHOLD)) {
            fall_state = POSTURE_CHECK_FALL;
        }
        else {
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == POSTURE_CHECK_FALL) {
        send_values_serial();
        Serial.println("IN POSTURE_CHECK_FALL");
        if(posture_check_angle()) {
            fall_state = DETECTED_FALL;
        }
        else {
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == DETECTED_FALL) {
        Serial.println("IN DETECTED_FALL");
        cv.fall_event_val = 1.0; // toggle fall signal and send
        send_values_serial();
        cv.fall_event_val = 0.0;

        // check if stationary post fall
        update_buffer();
        if(check_stationary()) {
            fall_state = STATIONARY_POST_FALL;
        }
        else {
            // go back to IDLE if moving again
            initialize_values();
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == STATIONARY_POST_FALL) {
        Serial.println("IN STATIONARY_POST_FALL");
        send_values_serial();
        update_buffer(1);
        // stay in this state if still unmoving
        if(check_stationary()) {
            fall_state = STATIONARY_POST_FALL;
        }
        // return to idle
        else {
            initialize_values();
            fall_state = IDLE_FALL;
        }
    }

    delay(LOOP_DELAY);
}
