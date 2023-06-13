// Line Following Robot using PID Control with L298N Motor Driver

// Pin connections
const int leftMotorPin1 = 2;
const int leftMotorPin2 = 3;
const int rightMotorPin1 = 4;
const int rightMotorPin2 = 5;
const int irSensorPins[] = {A0, A1, A2, A3, A4}; // IR sensor analog input pins

// PID Constants
const float Kp = 1.0;  // Proportional constant
const float Ki = 0.1;  // Integral constant
const float Kd = 0.05; // Derivative constant

// Setpoint (desired position on the line)
const int setpoint = 500;

// Variables
int sensorValues[5];
float error, lastError, totalError, correction;

void setup() {
  // Initialize motor control pins as output
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(irSensorPins[i]);
  }
  
  // Calculate error
  int weightedSum = 0;
  int totalWeight = 0;
  for (int i = 0; i < 5; i++) {
    weightedSum += sensorValues[i] * (i - 2);
    totalWeight += sensorValues[i];
  }
  error = setpoint - weightedSum / totalWeight;
  
  // PID calculations
  correction = Kp * error + Ki * totalError + Kd * (error - lastError);
  
  // Update last error and total error
  lastError = error;
  totalError += error;
  
  // Motor control
  int leftSpeed = 200;  // Base speed for left motor
  int rightSpeed = 200; // Base speed for right motor
  
  // Apply correction to motor speeds
  leftSpeed += correction;
  rightSpeed -= correction;
  
  // Limit motor speeds to avoid exceeding maximum values
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Set motor directions
  if (leftSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  
  if (rightSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  
  // Set motor speeds
  analogWrite(leftMotorPin1, abs(leftSpeed));
  analogWrite(rightMotorPin1, abs(rightSpeed));
  
  // Print sensor values and correction for debugging
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print(" | ");
  }
  Serial.print("Correction: ");
  Serial.println(correction);
  
  delay(10);
}

