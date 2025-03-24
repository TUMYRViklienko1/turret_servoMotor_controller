#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <math.h>

constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint16_t MAX_ANGLE = 180;
#define MAX_SERVO 2

uint8_t baseServo = 0;     // Pin for pan
uint8_t shoulderServo = 1; // Pin for tilt

iarduino_MultiServo SercoController;
uint16_t servoMotorAngles[MAX_SERVO]; // Array to hold angles

void getFromSerialPort() {
  String dataFromSerialPort = Serial.readStringUntil('\n');
  int valueIndex = 0; // Index for servoMotorAngles array
  int startIdx = 0;   // Start index for substring
  int commaIdx;

  // Loop to parse the string
  while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1 && valueIndex < MAX_SERVO) {
      servoMotorAngles[valueIndex] =  dataFromSerialPort.substring(startIdx, commaIdx).toInt();
      startIdx = commaIdx + 1;
      valueIndex++;
  }

  // Handle the last value
  if (valueIndex < MAX_SERVO) {
      servoMotorAngles[valueIndex] = dataFromSerialPort.substring(startIdx).toInt();
  }
}


void sendToServo(const uint16_t* servoMotorAngles) {
SercoController.servoWrite(baseServo, servoMotorAngles[0]);
SercoController.servoWrite(shoulderServo, servoMotorAngles[1]);
delay(10);
}

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(10); // Set timeout to 1000 milliseconds
    while (!Serial) {}

    // Servo initialization
    SercoController.servoSet(baseServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    SercoController.servoSet(shoulderServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);

    SercoController.begin();
    SercoController.servoWrite(baseServo, 90);
    SercoController.servoWrite(shoulderServo, 90);
}

void loop() {
    if (Serial.available()) {
        getFromSerialPort(); // Get coordinates from serial port
        sendToServo(servoMotorAngles);
    }
}

