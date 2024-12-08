#define LED           0
#define Sound_Out     A0 // --> pin 23 (PC0)

void setup() {
  // put your setup code here, to run once:
  pinMode(Sound_Out, INPUT);        
  pinMode(LED, OUTPUT);
}

int currentValue = 0;
int previousValue = analogRead(Sound_Out);
void loop() {
  // Wait until we detect a change in audio input
  currentValue = analogRead(Sound_Out);
  int change = abs(currentValue - previousValue);
  // Check if the change is sharp (value can be adjusted)
  if (change > 20) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  previousValue = currentValue;
}