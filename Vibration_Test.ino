#define LED           0
#define Vib_Out       4  // --> pin 6  (PD4)

void setup() {
  // put your setup code here, to run once:
  pinMode(Vib_Out, INPUT);  
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int vibData = digitalRead(Vib_Out);
  if (vibData == 1) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}