/*
  Arduino Mega 2560 + TB6600 stepper driver controlled by robot DO0.

  This program sends STEP and DIR signals from the Arduino Mega 2560 to
  a TB6600 stepper motor driver. The motor runs only when the robot DO0
  signal is active at the Arduino input pin.

  Inputs:
    DO0_INPUT_PIN: digital signal from robot DO0.

  Outputs:
    STEP_PIN: pulse signal to TB6600 PUL input.
    DIR_PIN: direction signal to TB6600 DIR input.
    ENABLE_PIN: enable signal to TB6600 ENA input.

  Units:
    Step pulse timing is in microseconds.
*/

// User Settings
// TODO: Feasible range is any digital pin on Arduino Mega 2560.
const int STEP_PIN                 = 3;

// TODO: Feasible range is any digital pin on Arduino Mega 2560.
const int DIR_PIN                  = 2;

// TODO: Feasible range is any digital pin on Arduino Mega 2560.
const int ENABLE_PIN               = 4;

// TODO: Connect robot DO0 signal here. Feasible range is digital pins.
const int DO0_INPUT_PIN            = 8;

// TODO: Set false if robot DO0 gives LOW when active.
const bool DO0_ACTIVE_HIGH         = true;

// TODO: Set false if your TB6600 enable logic is opposite.
const bool ENABLE_ACTIVE_LOW       = false;

// TODO: Use larger values for slower speed and easier first testing.
const unsigned int STEP_DELAY_US   = 800;

// TODO: Motor direction while DO0 is active.
const bool MOTOR_DIRECTION_HIGH    = true;


void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DO0_INPUT_PIN, INPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, MOTOR_DIRECTION_HIGH);
  disableDriver();
}


void loop()
{
  if (isDo0Active()) {
    enableDriver();
    moveStepperOneStep();
  }
  else {
    digitalWrite(STEP_PIN, LOW);
    disableDriver();
  }
}


void enableDriver()
{
  if (ENABLE_ACTIVE_LOW) {
    digitalWrite(ENABLE_PIN, LOW);
  }
  else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
}


void disableDriver()
{
  if (ENABLE_ACTIVE_LOW) {
    digitalWrite(ENABLE_PIN, HIGH);
  }
  else {
    digitalWrite(ENABLE_PIN, LOW);
  }
}


bool isDo0Active()
{
  int read_do0_value = digitalRead(DO0_INPUT_PIN);

  if (DO0_ACTIVE_HIGH) {
    return read_do0_value == HIGH;
  }

  return read_do0_value == LOW;
}


void moveStepperOneStep()
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}
