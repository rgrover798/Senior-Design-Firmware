#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#define LED 8

Adafruit_VL53L1X laserSensor = Adafruit_VL53L1X();

void setup() {
  pinMode(LED, OUTPUT);                // Set LED pin as output
  Wire.begin();
  laserSetupTest();
}                                 

void loop() {
  laserDistanceTest();

}


// INPUT TEST 3: Test to make sure laser has been setup and connected
void laserSetupTest() {
  if (!laserSensor.begin()) {
    digitalWrite(LED, HIGH);
  } else {
    // Blink LED three times if initialization succeeds
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
  }
  // Set the measurement timing budget (in ms)
  laserSensor.setTimingBudget(20000); // Set to 20ms
  // Set the inter-measurement period (in ms)
  laserSensor.VL53L1X_SetInterMeasurementInMs(50); // Set to 50ms
  // Start continuous ranging
  laserSensor.startRanging();
}


// INPUT TEST 4: Test to read laser distance from laser sensor
void laserDistanceTest() {
  for (int i = 0; i < 20; i++) { // Run the test for 20 readings (10 seconds)
    int distance = laserSensor.distance(); // Read the distance

    // Check if the distance is valid (non-zero)
    if (distance < 1000) { // Check if the sensor has a valid reading
      digitalWrite(LED, HIGH); // Light up the LED if the reading is valid
    } else {
      digitalWrite(LED, LOW); // Turn off the LED if no valid reading
    }
    
    delay(500); // Delay between readings (half a second)
  }
}