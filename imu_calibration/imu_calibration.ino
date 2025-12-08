#include "LSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// to calibrate gyroscope
#define CALIBRATE_SIZE 200 // number of samples to calibrate with

double cal_gx = 0.0;
double cal_gy = 0.0;
double cal_gz = 0.0;

double gx_data = 0.0;
double gy_data = 0.0;
double gz_data = 0.0;

double cal_ax = 0.0;
double cal_ay = 0.0;
double cal_az = 0.0;

double ax_data = 0.0;
double ay_data = 0.0;
double az_data = 0.0;


#define GYRO_VAR 0.004
#define ACCEL_VAR 0.0001

int num_samples = 0;
bool calibration_done = false;

struct Stats {
    static const int N = CALIBRATE_SIZE;
    double buf[N];
    int index = 0;
    int count = 0;
    double sum = 0;
    double sumsq = 0;

    void add(double x) {
        if (count < N) {
            buf[index] = x;
            sum += x;
            sumsq += x * x;
            count++;
        } else {
            // remove oldest, add newest
            sum -= buf[index];
            sumsq -= buf[index] * buf[index];
            buf[index] = x;
            sum += x;
            sumsq += x * x;
        }

        index = (index + 1) % N;
    }

    double mean() const {
        return (count > 0) ? sum / count : 0;
    }

    double variance() const {
        if (count < 2) return 0;
        double m = mean();
        return (sumsq / count) - (m * m);
    }

    bool full() const {
        return count == N;
    }
};

Stats gxsts, gysts, gzsts, axsts, aysts, azsts;

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
    gx_data = myIMU.readFloatGyroX();
    gy_data = myIMU.readFloatGyroY();
    gz_data = myIMU.readFloatGyroZ();

    gxsts.add(gx_data);
    gysts.add(gy_data);
    gzsts.add(gz_data);

}

void calibrate_accel_offsets() {
    ax_data = myIMU.readFloatAccelX();
    ay_data = myIMU.readFloatAccelY();
    az_data = myIMU.readFloatAccelZ();

    axsts.add(ax_data);
    aysts.add(ay_data);
    azsts.add(az_data);
}

void loop() {
    bool gyro_done = (num_samples >= CALIBRATE_SIZE) && 
                     (gxsts.variance() < GYRO_VAR) && 
                     (gysts.variance() < GYRO_VAR) && 
                     (gzsts.variance() < GYRO_VAR);
    
    bool accel_done = (num_samples >= CALIBRATE_SIZE) && 
                      (axsts.variance() < ACCEL_VAR) && 
                      (aysts.variance() < ACCEL_VAR) && 
                      (azsts.variance() < ACCEL_VAR);

    if(!calibration_done || (num_samples < CALIBRATE_SIZE)) {
        calibrate_gyro_offsets();
        calibrate_accel_offsets();

        Serial.println("Current variances");
        Serial.print("gx var: "); Serial.println(gxsts.variance(),6);
        Serial.print("gy var: "); Serial.println(gysts.variance(),6);
        Serial.print("gz var: "); Serial.println(gzsts.variance(),6);
        Serial.print("ax var: "); Serial.println(axsts.variance(),6);
        Serial.print("ay var: "); Serial.println(aysts.variance(),6);
        Serial.print("az var: "); Serial.println(azsts.variance(),6);

        num_samples = num_samples + 1;
    }

    if(!calibration_done && gyro_done && accel_done && (num_samples >= CALIBRATE_SIZE)) {
        calibration_done = true;
        cal_gx = gxsts.mean();
        cal_gy = gysts.mean();
        cal_gz = gzsts.mean();
        cal_ax = axsts.mean();
        cal_ay = aysts.mean();
        cal_az = azsts.mean();

        double g = sqrt(cal_ax*cal_ax + cal_ay*cal_ay + cal_az*cal_az);
        cal_az = cal_az - g;

        Serial.println("******Calibration is done**********");
        Serial.print("gx offset: "); Serial.println(cal_gx, 6);
        Serial.print("gy offset: "); Serial.println(cal_gy, 6);
        Serial.print("gz offset: "); Serial.println(cal_gz, 6);
        Serial.print("ax offset: "); Serial.println(cal_ax, 6);
        Serial.print("ay offset: "); Serial.println(cal_ay, 6);
        Serial.print("az offset: "); Serial.println(cal_az, 6);
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
