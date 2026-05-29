const int DO0_INPUT_PIN = 8;

void setup() {
  Serial.begin(9600);
  pinMode(DO0_INPUT_PIN, INPUT_PULLUP);
}

void loop() {
  int v = digitalRead(DO0_INPUT_PIN);
  Serial.println(v);   // expected: 1 when xArm output OFF, 0 when ON
  delay(200);
}