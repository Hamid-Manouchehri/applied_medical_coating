/*
  Arduino Mega 2560 + TB6600 serial keyboard control.

  This sketch receives simple serial commands from a PC Python script.
  It moves the stepper continuously forward, backward, or stops.

  Inputs:
    Serial command from PC:
      f = forward
      b = backward
      s = stop

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

// TODO: This should match the working PC-controlled sketch.
const bool ENABLE_ACTIVE_LOW       = true;

// TODO: Use larger values for slower speed and easier first testing.
const unsigned int STEP_DELAY_US   = 800;

// TODO: Serial baud rate. Must match the Python script.
const unsigned long SERIAL_BAUD    = 115200;

const int MOTOR_STOP               = 0;
const int MOTOR_FORWARD            = 1;
const int MOTOR_BACKWARD           = -1;

int motor_state                    = MOTOR_STOP;


void setup()
{
  Serial.begin(SERIAL_BAUD);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  enableDriver();

  Serial.println("TB6600 serial keyboard control ready.");
  Serial.println("Commands: f=forward, b=backward, s=stop");
}


void loop()
{
  readSerialCommand();

  if (motor_state == MOTOR_FORWARD) {
    digitalWrite(DIR_PIN, HIGH);
    moveStepperOneStep();
  }
  else if (motor_state == MOTOR_BACKWARD) {
    digitalWrite(DIR_PIN, LOW);
    moveStepperOneStep();
  }
  else {
    digitalWrite(STEP_PIN, LOW);
  }
}


void readSerialCommand()
{
  if (Serial.available() <= 0) {
    return;
  }

  char read_command = Serial.read();

  if (read_command == 'f') {
    motor_state = MOTOR_FORWARD;
    Serial.println("forward");
  }
  else if (read_command == 'b') {
    motor_state = MOTOR_BACKWARD;
    Serial.println("backward");
  }
  else if (read_command == 's') {
    motor_state = MOTOR_STOP;
    Serial.println("stop");
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


void moveStepperOneStep()
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}
