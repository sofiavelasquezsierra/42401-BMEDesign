#include <Wire.h>
#include <MAX30105.h>

MAX30105 sensor;
uint32_t last_ts = 0;
const uint32_t dt = 10000; // matches the sensor sampling frequency
uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  if (!sensor.begin(Wire, 400000)) {
    Serial.println("MAX30102 not found");
    while (1);
  }

  // Basic operational mode
  sensor.setup(
    30,   // LED power
    4,    // sample average
    2,    // red + IR
    100,  // 100 Hz
    411,  // pulse width
    4096  // ADC range
  );

  Serial.println("MAX30102 raw mode ready.");
}

void loop() {
    int n = sensor.check();

    if(n == 0) { // no samples to log
      last_ts = micros();
      counter = 0;
      return;
    }

    if(counter == 0) {
      last_ts = micros(); // realign timestamps to start of data burst
    }

    while (n--) {
      uint32_t ir_raw  = sensor.getFIFOIR();
      uint32_t red_raw = sensor.getFIFORed();

      if(counter > 0) {
        last_ts += dt; // fixed sampling interval to correct timestamps
      }

      Serial.print("IR=");
      Serial.print(ir_raw);
      Serial.print(",RED=");
      Serial.print(red_raw);
      Serial.print(",T=");
      Serial.println(last_ts);

      counter = counter + 1;
    }
}
