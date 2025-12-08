#include <Wire.h>
#include <MAX30105.h>

MAX30105 sensor;

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

    while (n--) {
      uint32_t ir_raw  = sensor.getFIFOIR();
      uint32_t red_raw = sensor.getFIFORed();
      uint32_t t = micros();

      Serial.print("IR=");
      Serial.print(ir_raw);
      Serial.print(",RED=");
      Serial.print(red_raw);
      Serial.print(",T=");
      Serial.println(t);
    }
}
