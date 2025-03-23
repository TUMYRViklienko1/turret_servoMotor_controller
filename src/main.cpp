#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <math.h>

constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint16_t MAX_ANGLE = 180;
#define MAX_SERVO 2
#define DURATION 1

uint8_t baseServo = 0;     // Pin for pan
uint8_t shoulderServo = 1; // Pin for tilt

iarduino_MultiServo SercoController;
uint16_t servoMotorAngles[MAX_SERVO]; // Array to hold angles
double previousAngles[MAX_SERVO] = {INITIAL_ANGLE, INITIAL_ANGLE};
int counterForServo = 0;

void getFromSerialPort() {
  String dataFromSerialPort = Serial.readStringUntil('\n');
  int valueIndex = 0; // Index for servoMotorAngles array
  int startIdx = 0;   // Start index for substring
  int commaIdx;

  // Loop to parse the string
  while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1 && valueIndex < MAX_SERVO) {
      servoMotorAngles[valueIndex] = 90 - dataFromSerialPort.substring(startIdx, commaIdx).toInt();
      startIdx = commaIdx + 1;
      valueIndex++;
  }

  // Handle the last value
  if (valueIndex < MAX_SERVO) {
      servoMotorAngles[valueIndex] =90 - dataFromSerialPort.substring(startIdx).toInt();
  }
}

double forwardKinematicBase(double theta_s, int theta_f, double tf) {
  static double endt = -1;
  static unsigned long ts;
  double a0, a1, a2, a3;
  double t;
  static double thet;

  if (endt != theta_f) {
      ts = millis();
      endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);

  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
      thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
      thet = theta_f;
  }

  return thet;
}

double forwardKinematicShoulder(double theta_s, int theta_f, double tf) {
  static double endt = -1;
  static unsigned long ts;
  double a0, a1, a2, a3;
  double t;
  static double thet;

  if (endt != theta_f) {
      ts = millis();
      endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);

  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
      thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
      thet = theta_f;
  }

  return thet;
}


void sendToServo(const uint16_t* servoMotorAngles) {
if (counterForServo == 0) {
    for (int i = 0; i < MAX_SERVO; i++) {
        previousAngles[i] = INITIAL_ANGLE;
    }
}

unsigned long startTime = millis();
unsigned long endTime = startTime + (DURATION * 1000);

while (millis() < endTime) {
    double newBaseAngle = forwardKinematicBase(previousAngles[0], servoMotorAngles[0], DURATION);
    double newShoulderAngle = forwardKinematicShoulder(previousAngles[1], servoMotorAngles[1], DURATION);

    SercoController.servoWrite(baseServo, newBaseAngle);
    SercoController.servoWrite(shoulderServo, newShoulderAngle);
    delay(10);
}

// Store the last angles
for (int i = 0; i < MAX_SERVO; ++i) {
    previousAngles[i] = servoMotorAngles[i];
}

counterForServo++;
}

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(10); // Set timeout to 1000 milliseconds
    while (!Serial) {}

    // Servo initialization
    SercoController.servoSet(baseServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    SercoController.servoSet(shoulderServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    SercoController.begin();
}

void loop() {
    if (Serial.available()) {
        getFromSerialPort(); // Get coordinates from serial port
        sendToServo(servoMotorAngles);
    }
}






