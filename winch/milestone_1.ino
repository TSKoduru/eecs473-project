#define stepPin 2
#define dirPin 5
#define enPin 8

// User-defined constants
const int stepsPerRev = 200;     // Full steps per revolution
const int moveSteps = 10000;       // Number of steps to move (e.g., 2 revs)
const int stepDelay = 250;      // Microseconds between steps (adjust for speed)
const int holdTime = 2000;       // Time to hold at top/bottom in milliseconds

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(enPin, LOW);  // Enable motor driver
}

void loop() {
  // Move up
  moveStepper(moveSteps, LOW);
  delay(holdTime); // Hold at the top

  // Move down
  moveStepper(moveSteps, HIGH);
  delay(holdTime); // Hold at the bottom
}

// Helper function to move the motor
void moveStepper(int steps, bool direction) {
  digitalWrite(dirPin, direction);

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}
