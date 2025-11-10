#include <Servo.h>

Servo esc;  // Create a Servo object for the ESC

void setup() {
  Serial.begin(9600);
  esc.attach(3);  // ESC control signal connected to pin 3

  // Initialize ESC at neutral (stop)
  esc.writeMicroseconds(1500);
  delay(5000);  // Give ESC time to initialize (usually beeps)
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') {
      esc.writeMicroseconds(1500);
    }
    if (c == 'o') {
      esc.writeMicroseconds(1600);
    }
  }
  
  // // Ramp up from neutral (1500 µs) to full forward (2000 µs)
  // for (int pulse = 1500; pulse <= 2000; pulse += 5) {
  //   esc.writeMicroseconds(pulse);
  //   delay(20); // smooth ramp (20ms between steps)
  // }

  // delay(1000); // hold at full forward for 1 second

  // // Ramp back down to neutral (1500 µs)
  // for (int pulse = 2000; pulse >= 1500; pulse -= 5) {
  //   esc.writeMicroseconds(pulse);
  //   delay(20);
  // }

  // delay(1000); // hold at neutral for 1 second

  // // Ramp down to full reverse (1000 µs)
  // for (int pulse = 1500; pulse >= 1000; pulse -= 5) {
  //   esc.writeMicroseconds(pulse);
  //   delay(20);
  // }

  // delay(1000); // hold at full reverse for 1 second

  // // Ramp back to neutral again
  // for (int pulse = 1000; pulse <= 1500; pulse += 5) {
  //   esc.writeMicroseconds(pulse);
  //   delay(20);
  // }

  // delay(2000); // pause before repeating the cycle
}
