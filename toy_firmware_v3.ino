#include <Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// Assign pins to ATMega328P pinout
#define LED 0
#define Speaker_In 3  // --> pin 5  (PD3)
#define Vib_Out 4     // --> pin 6  (PD4)
#define STEP_M1 5     // --> pin 11 (PD5)
#define STEP_M2 6     // --> pin 12 (PD6)
#define DIR_M2 7      // --> pin 13 (PD7)
#define DIR_M1 8      // --> pin 14 (PB0)
#define PWM_Tail 9    // --> pin 15 (PB1)
#define PWM_Treat 10  // --> pin 16 (PB2)
#define Sound_Out A0  // --> pin 23 (PC0)

// Create servo objects
Servo tailServo;
Servo treatServo;

// Create an instance of the VL53L1X sensor
Adafruit_VL53L1X laserSensor = Adafruit_VL53L1X();

void setup() {
  // Set motor control pins as outputs
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(STEP_M1, OUTPUT);
  pinMode(STEP_M2, OUTPUT);
  pinMode(PWM_Tail, OUTPUT);
  pinMode(PWM_Treat, OUTPUT);
  pinMode(LED, OUTPUT);

  // Set sensor-related pins as inputs
  pinMode(Sound_Out, INPUT);
  pinMode(Vib_Out, INPUT);

  // Other outputs
  tailServo.attach(PWM_Tail);    // Attach tail servo to its PIN
  treatServo.attach(PWM_Treat);  // Attach treat servo to its PIN
  pinMode(Speaker_In, OUTPUT);   // Speaker output
  Wire.begin();                  // I2C communication enabled for SDA and SCL pins automatically

  // Initialize laser sensor
  laserSensor.begin();
  laserSensor.setTimingBudget(20000);  // Set to 20000us = 20 ms
  laserSensor.startRanging();          // Start continuous ranging
}

enum State {
  STATE_INIT,
  STATE_MOVE,
  STATE_FAST_MOVE,
  STATE_ROTATE,
  STATE_DISPENSE,
  STATE_AUDIO,
  STATE_STOP
};

State curState = STATE_INIT;
State prevState = STATE_INIT;
unsigned long numCatches = 0;
unsigned long timeCaught = 0;
unsigned long numClaps = 0;
unsigned long timeClaps = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000;
int tailState = 0;
int currentValue = 0;
int previousValue = 0;

void loop() {
  switch (curState) {
    case STATE_INIT:
      {
        // Yellow LED high for start state
        digitalWrite(LED, HIGH);

        // Reset the current and previous audio inputs
        currentValue = 0;
        previousValue = analogRead(Sound_Out);

        // Wait until we detect a change in audio input
        while (true) {
          // Get current sound
          currentValue = analogRead(Sound_Out);

          // Check if the change is sharp (value can be adjusted)
          if (abs(currentValue - previousValue) > 40 && millis() - timeClaps > 25) {
            numClaps += 1;

            // If its been longer than two seconds since last clap, and there are 1 or 2 claps then reset to only 1 clap
            if (millis() - timeClaps >= 2000 && (numClaps == 1 || numClaps == 2)) {
              numClaps = 1;
            }

            // Update time since last clap
            timeClaps = millis();
          }

          // After two seconds of no clapping, determine the command
          if (millis() - timeClaps >= 2000) {
            // Move to movement state
            if (numClaps == 2) {
              delay(500);
              digitalWrite(DIR_M1, LOW);
              digitalWrite(STEP_M1, LOW);
              prevState = curState;
              curState = STATE_MOVE;
              numClaps = 0;
              break;
            }
            // Move to locate state
            else if (numClaps >= 3) {
              prevState = curState;
              curState = STATE_AUDIO;
              numClaps = 0;
              break;
            } else {
              numClaps = 0;
            }
          }
          previousValue = currentValue;
        }
      }
      break;

    case STATE_MOVE:
      {
        // Yellow LED low for default move state
        digitalWrite(LED, LOW);

        // Variable declaration
        timeCaught += 1;
        int distance = laserSensor.distance();
        int vibration = digitalRead(Vib_Out);

        // Laser detection
        if (distance > 0 && distance < 1000) {  // Distance threshold can be set lower, 4000 mm is max
          prevState = curState;
          curState = STATE_ROTATE;
          break;
        }

        // Check for single catch
        if (vibration == 1 && timeCaught >= 500) {  // adjust as needed
          timeCaught = 0;
          numCatches += 1;
        }

        // Check for 3 catches
        if (numCatches >= 3) {
          prevState = curState;
          curState = STATE_STOP;
          break;
        }

        // Set motors to go forward, and generate step pulses
        digitalWrite(DIR_M1, HIGH);
        digitalWrite(DIR_M2, HIGH);

        if (millis() % 30 > 15) {
          if (digitalRead(STEP_M1) == HIGH) {
            digitalWrite(STEP_M1, LOW);
            digitalWrite(STEP_M2, LOW);
          } else {
            digitalWrite(STEP_M1, HIGH);
            digitalWrite(STEP_M2, HIGH);
          }
        }

        // State updates
        prevState = curState;
        if (millis() % 2000 == 0) {
          curState = STATE_FAST_MOVE;
        }

        // Listen for stop command
        getSound();
      }
      break;

    case STATE_FAST_MOVE:
      {
        // Yellow LED indicates FAST MOVE state
        digitalWrite(LED, HIGH);

        // Variable declaration
        timeCaught += 1;
        int distance = laserSensor.distance();
        int vibration = digitalRead(Vib_Out);

        // Laser detection
        if (distance > 0 && distance < 1000) {  // Distance threshold can be set lower, 4000 mm is max
          prevState = curState;
          curState = STATE_ROTATE;
          break;
        }

        // Check for single catch
        if (vibration == 1 && timeCaught >= 750) {  // adjust as needed
          timeCaught = 0;
          numCatches += 1;
        }

        // Check for 3 catches
        if (numCatches >= 3) {
          prevState = curState;
          curState = STATE_STOP;
          break;
        }

        // Set motors to go forward, and generate step pulses
        digitalWrite(DIR_M1, HIGH);
        digitalWrite(DIR_M2, HIGH);

        if (millis() % 30 > 15) {
          if (digitalRead(STEP_M1) == HIGH) {
            digitalWrite(STEP_M1, LOW);
            digitalWrite(STEP_M2, LOW);
          } else {
            digitalWrite(STEP_M1, HIGH);
            digitalWrite(STEP_M2, HIGH);
          }
        }

        // State updates
        prevState = curState;
        if (millis() % 2000 == 0) {
          curState = STATE_MOVE;
        }

        // Listen for stop command
        getSound();
      }
      break;

    case STATE_STOP:
      {
        delay(500);
        // Stop the motors, set all step and direction pins to LOW
        digitalWrite(DIR_M1, LOW);
        digitalWrite(DIR_M2, LOW);
        digitalWrite(STEP_M1, LOW);
        digitalWrite(STEP_M2, LOW);

        // Check if need to move to dispense state, otherwise something went wrong and we ended up here
        if (numCatches >= 3) {
          numCatches = 0;
          prevState = curState;
          curState = STATE_DISPENSE;
          break;
        } else {
          prevState = curState;
          curState = STATE_STOP;
        }
      }
      break;

    case STATE_ROTATE:
      {
        // Set everything to LOW momentarily to stop the toy
        digitalWrite(DIR_M1, LOW);
        digitalWrite(DIR_M2, LOW);
        digitalWrite(STEP_M1, LOW);
        digitalWrite(STEP_M2, LOW);
        delay(500);

        // Rotate LED indicator
        digitalWrite(LED, HIGH);

        // Rotate in place until laser sensor is detecting a distance outside the threshold
        int rotateDistance = laserSensor.distance();
        digitalWrite(DIR_M1, HIGH);
        digitalWrite(DIR_M2, LOW);

        // Rotate the toy until nothing detected
        while (rotateDistance > 0 && rotateDistance < 1000) {
          if (millis() % 20 > 10) {
            if (digitalRead(STEP_M1) == HIGH) {
              digitalWrite(STEP_M1, LOW);
              digitalWrite(STEP_M2, LOW);
            } else {
              digitalWrite(STEP_M1, HIGH);
              digitalWrite(STEP_M2, HIGH);
            }
          }
          rotateDistance = laserSensor.distance();
        }

        // State updates
        prevState = curState;
        curState = STATE_MOVE;
      }
      break;

    case STATE_DISPENSE:
      {
        // Open the treat hatch for two seconds
        treatServo.writeMicroseconds(1000);  // Move to -45°
        delay(2000);
        treatServo.writeMicroseconds(1500);  // Move to 0°
        delay(1000);

        prevState = curState;
        curState = STATE_MOVE;
      }
      break;

    case STATE_AUDIO:
      {
        // Yellow LED low for audio state
        digitalWrite(LED, LOW);

        // Boolean for vibration detection
        bool vibrDetected = false;

        // Play audio tone and check for vibrations
        for (long freq = 500; freq <= 1000; freq += 1) {
          int vibration = digitalRead(Vib_Out);
          tone(Speaker_In, freq);
          if (vibration == 1) {
            vibrDetected = true;
          }
          delay(2);
        }
        noTone(Speaker_In);

        // If there's a vibration then change to initial state
        if (vibrDetected == true) {
          prevState = curState;
          curState = STATE_INIT;
        } else {
          prevState = curState;
          curState = STATE_AUDIO;
        }
      }
      break;

    // Something wrong happened if we reach here, but if we do, move to stop state
    default:
      {
        prevState = curState;
        curState = STATE_STOP;
      }
      break;
  }
}

// Nonblocking tail movement
void tailServoMovement() {
  unsigned long currentMillis = millis();  // Get the current time

  // Non-blocking tail servo movement
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the last movement time

    if (tailState == 0) {
      tailServo.writeMicroseconds(2000);  // Move to 45°
      tailState = 1;                      // Update state
    } else {
      tailServo.writeMicroseconds(1000);  // Move to -45°
      tailState = 0;                      // Update state
    }
  }
}

// Nonblocking stop state detection
void getSound() {
  currentValue = analogRead(Sound_Out);
  // Check if the change is sharp (value can be adjusted)
  if (abs(currentValue - previousValue) > 60 && millis() - timeClaps > 25) {
    numClaps += 1;
    if (millis() - timeClaps >= 2000 && (numClaps == 1 || numClaps == 2)) {
      numClaps = 1;
    }
    timeClaps = millis();
  }

  if (millis() - timeClaps >= 2000) {
    if (numClaps == 2) {
      delay(500);
      digitalWrite(DIR_M1, LOW);
      digitalWrite(STEP_M1, LOW);
      digitalWrite(DIR_M2, LOW);
      digitalWrite(STEP_M2, LOW);
      prevState = curState;
      curState = STATE_INIT;
      numClaps = 0;
    } else {
      numClaps = 0;
    }
  }

  previousValue = currentValue;
}