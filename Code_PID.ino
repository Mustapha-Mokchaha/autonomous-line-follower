#include <QTRSensors.h>
#define LED_BUILTIN 2

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID constants
const float Kp = 0.025; // Proportional gain
const float Kd = 0.16;  // Derivative gain

// Motor driver pins
const uint8_t enableA = 15; // Enable pin for left motor
const uint8_t enableB = 5;  // Enable pin for right motor
const uint8_t In1 = 17;     // In1 for left motor
const uint8_t In2 = 22;     // In2 for left motor
const uint8_t In3 = 4;     // In3 for right motor
const uint8_t In4 = 16;      // In4 for right motor

// QTR sensor pins
const uint8_t ledOn = 12;  // Emitter pin
const uint8_t qtrPins[] = {14, 27, 26, 25, 33, 32, 19, 21};

// Motor speeds
const int baseSpeed = 45; // Base speed
const int maxSpeed = 57;  // Maximum speed

int lastError = 0;

void setup() {
  // Configure QTR sensor
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setEmitterPin(ledOn);

  // Calibrate QTR sensors
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate calibration
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(1); // 1 ms delay
  }
  digitalWrite(LED_BUILTIN, LOW); // Turn off LED after calibration

  // Initialize motor pins
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the position of the line
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculate the error
  int error = position - 3500;

  // Calculate PID terms
  int proportional = error;
  int derivative = error - lastError;
  int turn = Kp * proportional + Kd * derivative;

  // Compute motor speeds
  int leftSpeed = baseSpeed - turn;
  int rightSpeed = baseSpeed + turn;

  // Ensure motor speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Set motor speed
  analogWrite(enableA, leftSpeed);
  analogWrite(enableB, rightSpeed);

  // Control motor direction for left motor
  if (leftSpeed > 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  } else {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }

  // Control motor direction for right motor (adjusted)
  if (rightSpeed > 0) {
    digitalWrite(In3, HIGH); // Forward motion for right motor
    digitalWrite(In4, LOW);
  } else {
    digitalWrite(In3, LOW); // Reverse motion for right motor
    digitalWrite(In4, HIGH);
  }

  // Update last error
  lastError = error;

  // Debugging output
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" Right Speed: ");
  Serial.println(rightSpeed);

  delay(10); //Delay for loop
}
