#include <QTRSensors.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
LiquidCrystal_I2C lcd(0x27, 16, 2); 

SoftwareSerial bluetooth(2, 3);

const int motorLeftPWM = 5;
const int motorLeftDir = 6;
const int motorRightPWM = 9;
const int motorRightDir = 10;

const int trigPin = 11;
const int echoPin = 12;

int nodeMatrix[2][2]; 
int binaryValue;      
int decimalValue;    

float Kp = 0.1, Ki = 0, Kd = 0;
int lastError = 0, integral = 0;

const int obstacleThreshold = 15;

void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.calibrate();

  lcd.begin();
  lcd.backlight();
  lcd.print("Starting...");
  bluetooth.begin(9600);
  bluetooth.println("Bluetooth Ready");
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  delay(1000);
}

void loop() {
  if (detectObstacle()) {
    avoidObstacle();
    return; 
  }

  // Line following using PID control
  int position = qtr.readLineWhite(sensorValues);
  int error = position - 3500; 
  int motorSpeed = PIDControl(error);
  setMotorSpeed(motorSpeed, motorSpeed);

  if (detectNode()) {
    decodeNode();
    displayNodeValue();
    sendNodeValueBluetooth();
    adjustSpeedOrAngle();
  }
  if (reachedFinish()) {
    lcd.clear();
    lcd.print("FINISH");
    bluetooth.println("FINISH");
    stopMotors();
    while (true);
  }
}
int PIDControl(int error) {
  integral += error;
  int derivative = error - lastError;
  int correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  return correction;
}
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPWM, abs(leftSpeed));
  analogWrite(motorRightPWM, abs(rightSpeed));
  digitalWrite(motorLeftDir, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(motorRightDir, rightSpeed > 0 ? HIGH : LOW);
}
void stopMotors() {
  analogWrite(motorLeftPWM, 0);
  analogWrite(motorRightPWM, 0);
}
bool detectNode() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 500) { 
      return true;
    }
  }
  return false;
}

// Decode the node into a 4-bit binary value
void decodeNode() {
  // Read the 2x2 matrix using the sensors
  nodeMatrix[0][0] = sensorValues[0] > 500 ? 1 : 0; // A
  nodeMatrix[0][1] = sensorValues[1] > 500 ? 1 : 0; // B
  nodeMatrix[1][0] = sensorValues[2] > 500 ? 1 : 0; // C
  nodeMatrix[1][1] = sensorValues[3] > 500 ? 1 : 0; // D
  binaryValue = (nodeMatrix[0][0] << 3) | (nodeMatrix[0][1] << 2) | (nodeMatrix[1][0] << 1) | nodeMatrix[1][1];

  decimalValue = binaryValue;
}

// Display node value on LCD
void displayNodeValue() {
  lcd.clear();
  lcd.print("Node: ");
  lcd.print(binaryValue, BIN);
  lcd.setCursor(0, 1);
  lcd.print("Dec: ");
  lcd.print(decimalValue);
}
void sendNodeValueBluetooth() {
  bluetooth.print("Node: ");
  bluetooth.print(binaryValue, BIN);
  bluetooth.print(" Dec: ");
  bluetooth.println(decimalValue);
}
void adjustSpeedOrAngle() {
  if (decimalValue == 0) {
    setMotorSpeed(100, 100); 
  } else {
    int speedLimit = decimalValue / 5; 
    setMotorSpeed(speedLimit, speedLimit);
  }
}

bool reachedFinish() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 500) { 
      return true;
    }
  }
  return false;
}
bool detectObstacle() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; 
  if (distance < obstacleThreshold) {
    return true;
  }
  return false;
}
void avoidObstacle() {
  lcd.clear();
  lcd.print("Obstacle Detected!");
  bluetooth.println("Obstacle Detected!");
  stopMotors();
  delay(1000);

  // Reverse and turn
  setMotorSpeed(-100, -100);
  delay(500);
  setMotorSpeed(-100, 100); 
  delay(500);
  lcd.clear();
  lcd.print("Resuming...");
  bluetooth.println("Resuming...");
}