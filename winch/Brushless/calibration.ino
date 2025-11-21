#include <Servo.h>

Servo esc;
int pwm = 1500;

void setup() {
  esc.attach(9);
  Serial.begin(9600);

  esc.writeMicroseconds(1500);
  delay(3000); // arming time
}

void loop() {
  // Sweep forward direction
  for (pwm = 1505; pwm <= 2000; pwm += 5) {
    esc.writeMicroseconds(pwm);
    Serial.println(pwm);
    delay(300);
  }

  // Stop motor
  esc.writeMicroseconds(1500);
  delay(2000);

  // Sweep reverse direction (optional)
  for (pwm = 1495; pwm >= 1000; pwm -= 5) {
    esc.writeMicroseconds(pwm);
    Serial.println(pwm);
    delay(300);
  }

  // Stop motor again
  esc.writeMicroseconds(1500);
  
  while (1);
}

/* CALIBRATION BELOW */

// #include <Servo.h>

// Servo esc;

// void setup() {
//   esc.attach(9);

//   Serial.begin(9600);
//   Serial.println("ESC Calibration Start");
  
//   // Send max throttle
//   Serial.println("Sending MAX throttle...");
//   esc.writeMicroseconds(2000);
//   delay(3000);

//   // Send min throttle
//   Serial.println("Sending MIN throttle...");
//   esc.writeMicroseconds(1000);
//   delay(3000);

//   Serial.println("Calibration done!");
// }

// void loop() {}
