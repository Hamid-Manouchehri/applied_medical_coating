/*
  Arduino Mega 2560 + TB6600 stepper driver controlled by robot DO0.

  This program sends STEP and DIR signals from the Arduino Mega 2560 to
  a TB6600 stepper motor driver. The motor runs only when the robot DO0
  signal is active at the Arduino input pin. It keeps moving continuously
  while DO0 is active.

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

// TODO: Use true if DO0 acts like a switch to GND.
const bool USE_INTERNAL_PULLUP     = true;

// TODO: With INPUT_PULLUP, active is usually LOW.
const bool DO0_ACTIVE_HIGH         = false;

// TODO: This should match the working PC-controlled sketch.
const bool ENABLE_ACTIVE_LOW       = true;

// TODO: Use larger values for slower speed and easier first testing.
const unsigned int STEP_DELAY_US   = 800;

// TODO: Motor direction while DO0 is active.
const bool MOTOR_DIRECTION_HIGH    = true;

// TODO: Serial Monitor baud rate.
const unsigned long SERIAL_BAUD    = 9600;

bool previous_do0_active           = false;
unsigned long previous_print_ms    = 0;


void setup()
{
  Serial.begin(SERIAL_BAUD);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  if (USE_INTERNAL_PULLUP) {
    pinMode(DO0_INPUT_PIN, INPUT_PULLUP);
  }
  else {
    pinMode(DO0_INPUT_PIN, INPUT);
  }

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, MOTOR_DIRECTION_HIGH);
  enableDriver();

  Serial.println("Arduino Mega 2560 TB6600 DO0 control ready.");
  Serial.println("Motor moves while DO0 is active.");
}


void loop()
{
  bool read_do0_active = isDo0Active();
  printDo0Status(read_do0_active);

  if (read_do0_active) {
    moveStepperOneStep();
  }
  else {
    digitalWrite(STEP_PIN, LOW);
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


void printDo0Status(bool read_do0_active)
{
  unsigned long current_time_ms = millis();

  if (read_do0_active != previous_do0_active) {
    Serial.print("DO0 active = ");
    Serial.println(read_do0_active);
    previous_do0_active = read_do0_active;
    previous_print_ms = current_time_ms;
  }

  if (current_time_ms - previous_print_ms >= 1000) {
    Serial.print("DO0 raw = ");
    Serial.print(digitalRead(DO0_INPUT_PIN));
    Serial.print(", DO0 active = ");
    Serial.println(read_do0_active);
    previous_print_ms = current_time_ms;
  }
}


void moveStepperOneStep()
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}
