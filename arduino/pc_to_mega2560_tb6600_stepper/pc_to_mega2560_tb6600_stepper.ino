/*
  Arduino Mega 2560 + TB6600 stepper driver test program.

  This program sends STEP and DIR signals from the Arduino Mega 2560 to
  a TB6600 stepper motor driver. It rotates the motor forward, pauses,
  rotates the motor backward, pauses, and repeats.

  Inputs:
    None during normal operation.

  Outputs:
    STEP_PIN: pulse signal to TB6600 PUL input.
    DIR_PIN: direction signal to TB6600 DIR input.
    ENABLE_PIN: enable signal to TB6600 ENA input.

  Units:
    Step pulse timing is in microseconds.
    Motion distance is in driver input pulses.
*/

// User Settings
// TODO: Feasible range is any digital pin on Arduino Mega 2560.
const int STEP_PIN = 3;

// TODO: Feasible range is any digital pin on Arduino Mega 2560.
const int DIR_PIN = 2;

// TODO: Feasible range is any digital pin on Arduino Mega 2560.
const int ENABLE_PIN = 4;

// TODO: Set false if your TB6600 enable logic is opposite.
const bool ENABLE_ACTIVE_LOW = true;  // true: on, false: off

// TODO: Use larger values for slower speed and easier first testing.
const unsigned int STEP_DELAY_US = 800;

// TODO: Number of input pulses per move.
const long STEPS_PER_MOVE = 1600;

// TODO: Pause time after each move, in milliseconds.
const unsigned long PAUSE_TIME_MS = 1000;


void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  enableDriver();
}


void loop() {
  digitalWrite(DIR_PIN, HIGH);
  moveStepper(STEPS_PER_MOVE);
  delay(PAUSE_TIME_MS);

  digitalWrite(DIR_PIN, LOW);
  moveStepper(STEPS_PER_MOVE);
  delay(PAUSE_TIME_MS);
}


void enableDriver() {
  if (ENABLE_ACTIVE_LOW) {
    digitalWrite(ENABLE_PIN, LOW);
  } else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
}


void disableDriver() {
  if (ENABLE_ACTIVE_LOW) {
    digitalWrite(ENABLE_PIN, HIGH);
  } else {
    digitalWrite(ENABLE_PIN, LOW);
  }
}


void moveStepper(long number_of_steps) {
  for (long step_count = 0; step_count < number_of_steps; step_count++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}
