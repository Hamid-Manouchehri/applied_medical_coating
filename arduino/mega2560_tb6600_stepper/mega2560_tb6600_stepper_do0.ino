/*
  Arduino Mega 2560 + TB6600 stepper driver controlled by xArm DO0/DO1.

  This program sends STEP and DIR signals from the Arduino Mega 2560 to
  a TB6600 stepper motor driver. xArm DO0 sets motor direction.
  xArm DO1 starts and stops continuous motor motion.

  Inputs:
    DO0_DIRECTION_PIN: digital signal from xArm DO0.
    DO1_RUN_PIN: digital signal from xArm DO1.

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

// TODO: Connect xArm DO0 direction signal here. Feasible range is digital.
const int DO0_DIRECTION_PIN        = 8;

// TODO: Connect xArm DO1 run/stop signal here. Feasible range is digital.
const int DO1_RUN_PIN              = 9;

// TODO: Use true if DO0 and DO1 act like switches to GND.
const bool USE_INTERNAL_PULLUP     = true;

// TODO: With INPUT_PULLUP, active is usually LOW for xArm outputs.
const bool XARM_OUTPUT_ACTIVE_HIGH = false;

// TODO: This should match the working PC-controlled sketch.
const bool ENABLE_ACTIVE_LOW       = true;

// TODO: Use larger values for slower speed and easier first testing.
const unsigned int STEP_DELAY_US   = 800;

// TODO: Direction pin value when DO0 is active.
const bool DO0_ACTIVE_DIR_HIGH     = true;

// TODO: Direction pin value when DO0 is inactive.
const bool DO0_INACTIVE_DIR_HIGH   = false;

// TODO: Serial Monitor baud rate.
const unsigned long SERIAL_BAUD    = 9600;

bool previous_do0_direction_active = false;
bool previous_do1_run_active       = false;
unsigned long previous_print_ms    = 0;


void setup()
{
  Serial.begin(SERIAL_BAUD);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  if (USE_INTERNAL_PULLUP) {
    pinMode(DO0_DIRECTION_PIN, INPUT_PULLUP);
    pinMode(DO1_RUN_PIN, INPUT_PULLUP);
  }
  else {
    pinMode(DO0_DIRECTION_PIN, INPUT);
    pinMode(DO1_RUN_PIN, INPUT);
  }

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, DO0_INACTIVE_DIR_HIGH);
  enableDriver();

  Serial.println("Arduino Mega 2560 TB6600 DO0/DO1 control ready.");
  Serial.println("DO0 controls direction. DO1 controls run/stop.");
}


void loop()
{
  bool read_do0_direction_active = isXarmOutputActive(DO0_DIRECTION_PIN);
  bool read_do1_run_active = isXarmOutputActive(DO1_RUN_PIN);

  setMotorDirection(read_do0_direction_active);
  printXarmStatus(read_do0_direction_active, read_do1_run_active);

  if (read_do1_run_active) {
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


bool isXarmOutputActive(int input_pin)
{
  int read_input_value = digitalRead(input_pin);

  if (XARM_OUTPUT_ACTIVE_HIGH) {
    return read_input_value == HIGH;
  }

  return read_input_value == LOW;
}


void setMotorDirection(bool read_do0_direction_active)
{
  if (read_do0_direction_active) {
    digitalWrite(DIR_PIN, DO0_ACTIVE_DIR_HIGH);
  }
  else {
    digitalWrite(DIR_PIN, DO0_INACTIVE_DIR_HIGH);
  }
}


void printXarmStatus(
  bool read_do0_direction_active,
  bool read_do1_run_active
)
{
  unsigned long current_time_ms = millis();

  if (read_do0_direction_active != previous_do0_direction_active) {
    Serial.print("DO0 direction active = ");
    Serial.println(read_do0_direction_active);
    previous_do0_direction_active = read_do0_direction_active;
    previous_print_ms = current_time_ms;
  }

  if (read_do1_run_active != previous_do1_run_active) {
    Serial.print("DO1 run active = ");
    Serial.println(read_do1_run_active);
    previous_do1_run_active = read_do1_run_active;
    previous_print_ms = current_time_ms;
  }

  if (current_time_ms - previous_print_ms >= 1000) {
    Serial.print("DO0 raw = ");
    Serial.print(digitalRead(DO0_DIRECTION_PIN));
    Serial.print(", DO0 direction active = ");
    Serial.print(read_do0_direction_active);
    Serial.print(", DO1 raw = ");
    Serial.print(digitalRead(DO1_RUN_PIN));
    Serial.print(", DO1 run active = ");
    Serial.println(read_do1_run_active);
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
