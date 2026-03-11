#include "LSM6DS3.h"
#include "Wire.h"
#include <bluefruit.h>
#include <math.h>


// constants
const float G = 9.81;
const float RAD_TO_DEG_CONV = 57.295779;

// macros

// should run at 100Hz
#define LOOP_DELAY 10 

// These values are inspired by the paper
#define BUF_SIZE 200
#define BUF_SMALL 100
#define IDLE_TRIGGER 0.85 // 0.8 in paper but increased to allow for walk/run detection
#define CHECK_TRIGGER 1.4

// These values are inspired by the data
#define ACCEL_DEV_THRESHOLD 0.06 // 0.1 in the paper
#define GYRO_DEV_THRESHOLD 16.3 // 10 in the paper
#define DEV_BUFFER_SIZE 50 // same as paper

// Thresholds for detecting walking or running
#define ACCEL_DEV_WALKING 0.13 // from the data
#define ACCEL_DEV_RUNNING 0.4 // from the data but need more trials
#define WALKING_SPM_MIN 75
#define RUNNING_SPM_MIN 150
#define PEAK_BUF_SIZE 10


// straight comparison to 60 degrees in the paper
#define TILT_TRIGGER_ANGLE 60
// relative comparison
#define TILT_TRIGGER 0.3
#define INIT_TILT_SIZE 20
#define FINAL_TILT_SIZE 80
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

// motion classifer states, sorry for the bad naming
enum FALL_STATES {
    IDLE_FALL = 0,
    CHECK_FALL = 1,
    STABILIZE_FALL = 2,
    POSTURE_CHECK_FALL = 3,
    DETECTED_FALL = 4,
    STATIONARY_POST_FALL = 5,
    WALKING = 6,
    RUNNING = 7,
    JUMPING_OR_QUICK_SIT = 8
} fall_state;

String fall_state_strings[9] = {"IDLE_FALL", "CHECK_FALL", "STABILIZE_FALL", "POSTURE_CHECK_FALL", "DETECTED_FALL", "STATIONARY_POST_FALL", "WALKING", "RUNNING", "JUMPING_OR_QUICK_SIT"};

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
        // update_values(1);
        send_values_serial();
        if(cv.A_SVM >= CHECK_TRIGGER) {
            large_accel = true;
        }
        delay(LOOP_DELAY); // delay since collecting samples
    }
    return large_accel;
}

// Calculate stdev over the last DEV_BUFFER_SIZE samples of the buffer
// If doesn't work, consider doing a comparison between earlier samples
// and later samples to show stabilization
float std_dev_check(IMU_COMP dev_type, int buffer_size) {
    float mean = 0.0;
    int start_idx = 0;
    int end_idx = 0;
    if(buffer_size == BUF_SIZE) {
        start_idx = BUF_SIZE - DEV_BUFFER_SIZE;
        end_idx = BUF_SIZE;
        buffer_size = DEV_BUFFER_SIZE;
    }
    else {
        start_idx = 0;
        end_idx = buffer_size;
    }
    // naive implementation, optimize if lag is too bad
    for(int i = start_idx; i < end_idx; i++) {
        float val = (dev_type == ACCEL) ? asvm_buf[i] : gsvm_buf[i];
        mean = mean + val;
    }
    
    mean = mean/buffer_size;

    float variance = 0.0;

    for(int i = start_idx; i < end_idx; i++) {
        float val = (dev_type == ACCEL) ? asvm_buf[i] : gsvm_buf[i];
        variance = variance + (val - mean)*(val-mean);

    }

    variance = variance/buffer_size;
    
    return sqrt(variance);
    
}

// really just a angle percent difference check
bool posture_check() {
    float tilt_diff = 0.0;
    float tilt_init_sum = 0.0;
    float tilt_final_sum = 0.0;
    float hor_dist = 0.0;

    // extremely naive implementation, optimize later
    for(int i = 0; i < INIT_TILT_SIZE; i++) {
        hor_dist = sqrt(ax_buf[i]*ax_buf[i] + az_buf[i]*az_buf[i]);
        tilt_init_sum += atan2(ay_buf[i], hor_dist);
    }
    for(int i = BUF_SIZE - FINAL_TILT_SIZE; i < BUF_SIZE; i++) {
        hor_dist = sqrt(ax_buf[i]*ax_buf[i] + az_buf[i]*az_buf[i]);
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
        hor_dist = sqrt(ax_buf[i]*ax_buf[i] + az_buf[i]*az_buf[i]);
        tilt_final_sum += atan2(ay_buf[i], hor_dist);
    }

    float tilt_final_angle = fabs(((tilt_final_sum*RAD_TO_DEG_CONV)/FINAL_TILT_SIZE));
    Serial.print("Calculated angle: "); Serial.print(tilt_final_angle); Serial.print(", TILT_TRIGGER: "); Serial.println(TILT_TRIGGER_ANGLE);
    cv.fall_event_val = tilt_final_angle;
    return (tilt_final_angle <= TILT_TRIGGER_ANGLE);
}

void update_buffer(int buffer_size) {
    // reset update position
    update_pos = 0;
    for(int i = 0; i < buffer_size; i++) {
        update_values(1);
    }
    return;
}

bool check_stationary() {
    bool stationary = false;
    float sum = 0.0;
    for(int i = 0; i < BUF_SMALL; i++) {
        sum = sum + asvm_buf[i];
    }

    float avg_asvm = sum / BUF_SMALL;
    // ASVM = 1 when stationary
    stationary = (abs(avg_asvm - 1) <= STATIONARY_THRESHOLD);
    return stationary;
}

// calculate steps per minute over the window
float check_cadence() {
    float period = 0.0;
    float peak[PEAK_BUF_SIZE];
    float ts[PEAK_BUF_SIZE];

    // find peaks
    // average peak to take out local 
    return period;
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
        cv.fall_event_val = 0.0;
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
        // collect buffer of samples and send
        if(check_fall()) {
            fall_state = STABILIZE_FALL;
        }
        else {
            fall_state = IDLE_FALL;
        }
    }

    if(fall_state == STABILIZE_FALL) {
        send_values_serial();
        Serial.println("IN STABLIZE_ACCEL_FALL");
        // calculate standard deviations over the buffer
        float std_accel = std_dev_check(ACCEL, BUF_SIZE);
        float std_gyro = std_dev_check(GYRO, BUF_SIZE);
        cv.fall_event_val = std_accel;

        // sufficiently stabilized
        if(std_accel <= ACCEL_DEV_THRESHOLD && std_gyro <= GYRO_DEV_THRESHOLD) {
            fall_state = POSTURE_CHECK_FALL;
        }

        // walking threshold, only care about the accel std
        else if((std_accel >= ACCEL_DEV_WALKING) && 
                (std_accel <= ACCEL_DEV_RUNNING))
        {
            fall_state = WALKING;
        }

        // running threshold
        else if(std_accel >= ACCEL_DEV_RUNNING) {
            fall_state = RUNNING;
        }

        // weird other movement I guess
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
            fall_state = JUMPING_OR_QUICK_SIT;
        }
    }

    if(fall_state == DETECTED_FALL) {
        Serial.println("IN DETECTED_FALL");
        cv.fall_event_val = 1.0; // toggle fall signal and send
        send_values_serial();
        cv.fall_event_val = 0.0;

        // check if stationary post fall
        update_buffer(BUF_SMALL);
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
        update_buffer(BUF_SMALL);
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

    if(fall_state == WALKING || fall_state == RUNNING) {
        send_values_serial();

        // probably stay ready to detect fall
        if(cv.A_SVM <= IDLE_TRIGGER) {
            fall_state = CHECK_FALL;
        }

        // generate a small window of data to look at and analyze
        else {
            update_buffer(BUF_SMALL);

            // stabilized values, go back to idle
            if(std_dev_check(ACCEL, BUF_SMALL) <= ACCEL_DEV_THRESHOLD) {
                fall_state = IDLE_FALL;
            }

            // moving somehow
            else {
                float spm = check_cadence();
                // walking range
                if(spm <= RUNNING_SPM_MIN && spm >= WALKING_SPM_MIN) {
                    fall_state = WALKING;
                }
                // running range
                else if(spm >= RUNNING_SPM_MIN) {
                    fall_state = RUNNING;
                }
                
                // something else??
                else {
                    fall_state = IDLE_FALL;
                }
            }
        }
    }

    if(fall_state == JUMPING_OR_QUICK_SIT) {
        Serial.println("IN JUMPING_OR_QUICK_SIT");
        send_values_serial();
        fall_state = IDLE_FALL;
    }

    delay(LOOP_DELAY);
}
