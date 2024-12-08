#define STEP_M1       5  // --> pin 11 (PD5) ORDER BELOW IS CONFUSING, BUT CORRECT
#define STEP_M2       6  // --> pin 12 (PD6)
#define DIR_M2        7  // --> pin 13 (PD7)
#define DIR_M1        8  // --> pin 14 (PB0)
#define LED           0

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(STEP_M1, OUTPUT);
  pinMode(STEP_M2, OUTPUT);

  digitalWrite(DIR_M1, HIGH);
  digitalWrite(DIR_M2, HIGH);
}

void loop() {
  for (int i = 0; i < 200; i++) { // Move 200 steps
    digitalWrite(STEP_M1, HIGH);
    delayMicroseconds(2500);
    digitalWrite(STEP_M1, LOW);
    delayMicroseconds(2500);
  }
  for (int i = 0; i < 200; i++) { // Move 200 steps
    digitalWrite(STEP_M2, HIGH);
    delayMicroseconds(2500);
    digitalWrite(STEP_M2, LOW);
    delayMicroseconds(2500);
  }
}
