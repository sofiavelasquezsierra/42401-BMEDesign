#include <Wire.h>
#include <MAX30105.h>

MAX30105 sensor;

// State variables for 0.4–4.0 Hz band-pass filter (biquad)
float b0 = 0.06745527;
float b1 = 0.0;
float b2 = -0.06745527;
float a1 = -1.64745998;
float a2 = 0.70551987;

float ir_x1 = 0, ir_x2 = 0;
float ir_y1 = 0, ir_y2 = 0;

float red_x1 = 0, red_x2 = 0;
float red_y1 = 0, red_y2 = 0;

// DC smoothing filter
float irDC = 0;
float redDC = 0;
float alphaDC = 0.01;   // slow LPF to track DC baseline

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting MAX30102 calibration mode...");

  if (!sensor.begin(Wire, 400000)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }

  // Sensor configuration for ML calibration
  sensor.setup(
    30,     // LED brightness
    4,      // sample average
    2,      // red + IR mode
    100,    // sample rate (Hz)
    411,    // pulse width
    4096    // ADC range
  );

  Serial.println("MAX30102 ready.");
}

// Applies a biquad band-pass filter to an input sample
float applyFilter(float x, float &x1, float &x2, float &y1, float &y2) {
  float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

  // Shift history
  x2 = x1;
  x1 = x;
  y2 = y1;
  y1 = y;

  return y;
}

void loop() {
  // Read from FIFO when new data is available
  if (sensor.check() > 0) {

    uint32_t ir_raw  = sensor.getFIFOIR();
    uint32_t red_raw = sensor.getFIFORed();

    float ir = (float)ir_raw;
    float red = (float)red_raw;

    // Band-pass filter around heart-rate frequency (0.4–4 Hz)
    float ir_filt  = applyFilter(ir, ir_x1, ir_x2, ir_y1, ir_y2);
    float red_filt = applyFilter(red, red_x1, red_x2, red_y1, red_y2);

    // Slow low-pass filter for DC baseline
    irDC  = (1 - alphaDC) * irDC  + alphaDC * ir;
    redDC = (1 - alphaDC) * redDC + alphaDC * red;

    // AC component magnitude from filtered signal
    float irAC  = fabs(ir_filt);
    float redAC = fabs(red_filt);

    // Ratio-of-ratios used for SpO2 estimation
    float R = 0.0;
    if (irAC > 0 && redAC > 0 && irDC > 0 && redDC > 0) {
      float redRatio = redAC / redDC;
      float irRatio  = irAC / irDC;
      R = redRatio / irRatio;
    }

    // Output all features for Python ML pipeline
    Serial.print("IR=");
    Serial.print(ir_raw);
    Serial.print(",RED=");
    Serial.print(red_raw);
    Serial.print(",IR_AC=");
    Serial.print(irAC, 6);
    Serial.print(",IR_DC=");
    Serial.print(irDC, 6);
    Serial.print(",RED_AC=");
    Serial.print(redAC, 6);
    Serial.print(",RED_DC=");
    Serial.print(redDC, 6);
    Serial.print(",R=");
    Serial.print(R, 6);
    Serial.print(",T=");
    Serial.println(millis());
  }
}
