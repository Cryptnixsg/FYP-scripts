#include "PID.h"
#include "AltitudeControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "FlightControlProcessor.h"
#include "Compass.h"
#include "AeroQuad.h"
#include "DataStorage.h"
#include "Motors_PWM.h"
#include "Gyroscope_MPU6000.h"
#include <AQMath.h>
#include <Wire.h>
#include "FourtOrderFilter.h"
#include "UserConfiguration.h"

// Pin assignments
#define MOTOR_PIN1 9
#define MOTOR_PIN2 10
#define MOTOR_PIN3 11
#define MOTOR_PIN4 12

#define BATTERY_PIN A0
#define UPPER_ULTRA_TRIG_PIN 3
#define UPPER_ULTRA_ECHO_PIN 4
#define LOWER_ULTRA_TRIG_PIN 5
#define LOWER_ULTRA_ECHO_PIN 6

// Receiver defines
#define RECEIVER_328P  // Used in AeroQuad_v1 and AeroQuadMini
#define RECEIVER_MEGA  // Used in AeroQuadMega_v1

// Motor PWM definition
#define MOTOR_PWM  // Standard PWM motor control

// Altitude Hold (Baro) and Range Finder
#ifdef AltitudeHoldBaro
  #define BMP085  // Barometric sensor
  vehicleState |= ALTITUDEHOLD_ENABLED;
#endif

#ifdef AltitudeHoldRangeFinder
  #define XLMAXSONAR  // MaxSonar for range finding
#endif

// Heading Hold using Compass
#ifdef HeadingMagHold
  #include <Compass.h>
  #define HMC5843  // Magnetometer model for heading hold
#endif

// Gyroscope and accelerometer objects
Gyroscope_MPU6000 gyro;
AcceleroM accel;

// PID control setup
PID altitudePID(1.0, 0.05, 0.1);  // Tuned for altitude control

// Ultrasonic Sensor Function for distance measurement
long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;  // Distance in cm
}

// Function to compare and store calibration data
void checkAndStoreCalibration() {
  float calibratedGyroBias = gyro.getBias();
  float storedGyroBias = readEEPROM();

  if (calibratedGyroBias != storedGyroBias) {
    writeEEPROM(calibratedGyroBias);  // Update stored value if different
  }

  float calibratedAccelBias = accel.getBias();
  float storedAccelBias = readEEPROM();

  if (calibratedAccelBias != storedAccelBias) {
    writeEEPROM(calibratedAccelBias);
  }
}

// Function to control motors with PID
void startMotorsWithPID() {
  long lowerDistance = readUltrasonicDistance(LOWER_ULTRA_TRIG_PIN, LOWER_ULTRA_ECHO_PIN);
  float desiredAltitude = 100.0;  // Example target altitude
  float currentAltitude = lowerDistance;
  float motorSpeed = altitudePID.compute(desiredAltitude, currentAltitude);

  analogWrite(MOTOR_PIN1, motorSpeed);
  analogWrite(MOTOR_PIN2, motorSpeed);
  analogWrite(MOTOR_PIN3, motorSpeed);
  analogWrite(MOTOR_PIN4, motorSpeed);
}

// Hover function at altitude set by upper ultrasonic sensor
void hoverAtAltitude() {
  long upperDistance = readUltrasonicDistance(UPPER_ULTRA_TRIG_PIN, UPPER_ULTRA_ECHO_PIN);
  float hoverAltitude = upperDistance;

  while (true) {
    float currentAltitude = readUltrasonicDistance(LOWER_ULTRA_TRIG_PIN, LOWER_ULTRA_ECHO_PIN);
    float motorSpeed = altitudePID.compute(hoverAltitude, currentAltitude);

    analogWrite(MOTOR_PIN1, motorSpeed);
    analogWrite(MOTOR_PIN2, motorSpeed);
    analogWrite(MOTOR_PIN3, motorSpeed);
    analogWrite(MOTOR_PIN4, motorSpeed);

    rotateLeft();

    if (holdingCommand && (millis() - holdStartTime) < 20000) {
      holdPosition(hoverAltitude);
    }

    resumeScanning();
  }
}

// Rotate the drone left
void rotateLeft() {
  analogWrite(MOTOR_PIN1, 150);
  analogWrite(MOTOR_PIN2, 255);
  analogWrite(MOTOR_PIN3, 150);
  analogWrite(MOTOR_PIN4, 255);
  delay(100);
}

// Function to hold position for a set time
void holdPosition(float altitude) {
  while ((millis() - holdStartTime) < 20000) {
    float currentAltitude = readUltrasonicDistance(LOWER_ULTRA_TRIG_PIN, LOWER_ULTRA_ECHO_PIN);
    float motorSpeed = altitudePID.compute(altitude, currentAltitude);

    analogWrite(MOTOR_PIN1, motorSpeed);
    analogWrite(MOTOR_PIN2, motorSpeed);
    analogWrite(MOTOR_PIN3, motorSpeed);
    analogWrite(MOTOR_PIN4, motorSpeed);
  }
}

// Measure critical sensors
void measureCriticalSensors() {
  measureGyro();  // General gyro measurement

#ifdef AltitudeHoldBaro
  measureBaroSum();  // Measures barometric altitude
#endif

#ifdef HeadingMagHold
  measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
  calculateHeading();  // Computes heading
#endif
}

// Function to resume environment scanning
void resumeScanning() {
  rotateLeft();
}

// Communication handler for commands from Raspberry Pi
void handlePiCommand(String command) {
  if (command == "hold") {
    holdingCommand = true;
    holdStartTime = millis();
  } else {
    holdingCommand = false;
  }
}

// FollowMe command for directional control based on Raspberry Pi input
void followMeCommand(String command) {
  if (command == "left") {
    // Adjust motor speeds to move left
    analogWrite(MOTOR_PIN1, 150);
    analogWrite(MOTOR_PIN2, 255);
    analogWrite(MOTOR_PIN3, 255);
    analogWrite(MOTOR_PIN4, 150);
  } else if (command == "right") {
    // Adjust motor speeds to move right
    analogWrite(MOTOR_PIN1, 255);
    analogWrite(MOTOR_PIN2, 150);
    analogWrite(MOTOR_PIN3, 150);
    analogWrite(MOTOR_PIN4, 255);
  } else if (command == "forward") {
    // Adjust motor speeds to move forward
    analogWrite(MOTOR_PIN1, 255);
    analogWrite(MOTOR_PIN2, 255);
    analogWrite(MOTOR_PIN3, 150);
    analogWrite(MOTOR_PIN4, 150);
  } else if (command == "backward") {
    // Adjust motor speeds to move backward
    analogWrite(MOTOR_PIN1, 150);
    analogWrite(MOTOR_PIN2, 150);
    analogWrite(MOTOR_PIN3, 255);
    analogWrite(MOTOR_PIN4, 255);
  } else if (command == "up") {
    // Increase motor speeds to ascend
    analogWrite(MOTOR_PIN1, 255);
    analogWrite(MOTOR_PIN2, 255);
    analogWrite(MOTOR_PIN3, 255);
    analogWrite(MOTOR_PIN4, 255);
  } else if (command == "down") {
    // Decrease motor speeds to descend
    analogWrite(MOTOR_PIN1, 100);
    analogWrite(MOTOR_PIN2, 100);
    analogWrite(MOTOR_PIN3, 100);
    analogWrite(MOTOR_PIN4, 100);
  } else if (command == "stop") {
    // Stop the drone (hover)
    analogWrite(MOTOR_PIN1, 200);
    analogWrite(MOTOR_PIN2, 200);
    analogWrite(MOTOR_PIN3, 200);
    analogWrite(MOTOR_PIN4, 200);
  }
}

void setup() {
  Serial.begin(115200);
  gyro.initialize();
  accel.initialize();

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);
  pinMode(UPPER_ULTRA_TRIG_PIN, OUTPUT);
  pinMode(UPPER_ULTRA_ECHO_PIN, INPUT);
  pinMode(LOWER_ULTRA_TRIG_PIN, OUTPUT);
  pinMode(LOWER_ULTRA_ECHO_PIN, INPUT);

  checkAndStoreCalibration();
}

void loop() {
  measureCriticalSensors();
  startMotorsWithPID();
  hoverAtAltitude();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handlePiCommand(command);  // Handle hold commands
    followMeCommand(command);  // Handle FollowMe commands
  }

  calculateFlightError();
  processThrottleCorrection();
  processHardManuevers();
  processMinMaxCommand();
  processFlightControl();
}

#ifdef AltitudeHoldRangeFinder
void updateRangeFinders() {
  // Update ultrasonic range finders
}
#endif
