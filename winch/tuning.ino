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

  Serial.println("\n=== Winch PWM Tuning Tool ===");
  Serial.println("Commands:");
  Serial.println("  1: freefall");
  Serial.println("  2: accel up");
  Serial.println("  3: decel up");
  Serial.println("  4: accel down");
  Serial.println("  5: decel down");
  Serial.println("  6: hold");
  Serial.println("================================\n");
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
    esc.writeMicroseconds(PWM_ACCEL_DOWN);
    Serial.println("→ Freefall (PWM_ACCEL_DOWN)");
  }
    else if (cmd == "2") {
        esc.writeMicroseconds(PWM_ACCEL_UP);
        Serial.println("→ Accelerate Up (PWM_ACCEL_UP)");
    }
    else if (cmd == "3") {
        esc.writeMicroseconds(PWM_DECEL_UP);
        Serial.println("→ Decelerate Up (PWM_DECEL_UP)");
    }
    else if (cmd == "4") {
        esc.writeMicroseconds(PWM_ACCEL_DOWN);
        Serial.println("→ Accelerate Down (PWM_ACCEL_DOWN)");
    }
    else if (cmd == "5") {
        esc.writeMicroseconds(PWM_DECEL_DOWN);
        Serial.println("→ Decelerate Down (PWM_DECEL_DOWN)");
    }
    else if (cmd == "6") {
        esc.writeMicroseconds(PWM_HOLD);
        Serial.println("→ Hold (PWM_HOLD)");
    }
    else {
        Serial.println("Unknown command. Please enter a number between 1 and 6.");
    }
}