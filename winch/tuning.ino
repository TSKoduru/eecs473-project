/* ================================
   SIMPLE PWM TUNING TOOL FOR WINCH
   ================================
   - Outputs PWM to an ESC using servo library
   - Lets you type commands over serial to test
   - Edit the defines at the top to tune behavior
*/

#include <Servo.h>
Servo esc;

// === EDIT THESE TO TUNE YOUR SYSTEM ===
#define PWM_NEUTRAL     1500
#define PWM_ACCEL_UP    1000
#define PWM_DECEL_DOWN  1100
#define PWM_HOLD        1200
#define PWM_DECEL_UP    1300
#define PWM_ACCEL_DOWN  1400

// =======================================

String command = "";

void setup() {
  Serial.begin(115200);
  esc.attach(9);   // ESC signal pin
  delay(2000);     // Allow ESC to arm

  esc.writeMicroseconds(PWM_NEUTRAL);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      handleCommand(command);
      command = "";
    } 
    else {
      command += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd == "1") {
    esc.writeMicroseconds(1500);
    Serial.println("→ 1500");
  }
    else if (cmd == "2") {
        esc.writeMicroseconds(1510);
        Serial.println("→ 1510");
    }
    else if (cmd == "3") {
        esc.writeMicroseconds(1520);
        Serial.println("→ 1520");
    }
    else if (cmd == "4") {
        esc.writeMicroseconds(1530);
        Serial.println("→ 1530");
    }
    else if (cmd == "5") {
        esc.writeMicroseconds(1540);
        Serial.println("→ 1540");
    }
    else if (cmd == "6") {
        esc.writeMicroseconds(1550);
        Serial.println("→ 1550");
    }
    else {
        Serial.println("Unknown command. Please enter a number between 1 and 6.");
    }
}