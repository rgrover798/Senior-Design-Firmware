#define Speaker_In 3
#define LED 0

void setup() {
  pinMode(LED, OUTPUT);             // Set LED pin as output
  pinMode(Speaker_In, OUTPUT);      // Speaker output
}                                 

void loop() {
  speakerSweepTest();
}

void speakerSweepTest() {
  for (int freq = 500; freq <= 1000; freq += 50) {
    digitalWrite(LED, HIGH);
    delay(100);
    tone(Speaker_In, freq);  // Output tone to speaker
    digitalWrite(LED, LOW);
    delay(100);
  }

  noTone(Speaker_In);  // Turn off speaker after sweep
}
