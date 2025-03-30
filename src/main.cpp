#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <math.h>

constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint16_t MAX_ANGLE = 180;
#define MAX_SERVO 2
#define HEADDEGREE 2

uint8_t baseServo = 0;     // Pin for pan
uint8_t shoulderServo = 1; // Pin for tilt

iarduino_MultiServo ServoController;
uint16_t headCoordinates[HEADDEGREE];



void pidControl();
void sendToServo(uint16_t panAngle, uint16_t tiltAngle);
void getFromSerialPort();

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(10);
    while (!Serial) {}

    // Servo initialization
    ServoController.servoSet(baseServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    ServoController.servoSet(shoulderServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    ServoController.begin();
    
    
    ServoController.servoWrite(baseServo, INITIAL_ANGLE);
    ServoController.servoWrite(shoulderServo, INITIAL_ANGLE);

}

void loop() {
    if (Serial.available()) {
        getFromSerialPort();
        pidControl();
    }
}

void getFromSerialPort() {
    String dataFromSerialPort = Serial.readStringUntil('\n');
    int x, y;

    if (sscanf(dataFromSerialPort.c_str(), "%d,%d", &x, &y) == 2) {
        headCoordinates[0] = x;
        headCoordinates[1] = y;
    }
}


void sendToServo(uint16_t panAngle, uint16_t tiltAngle) {
    uint16_t panAngleMap = map(panAngle, 0, 640, 0, 180);
    uint16_t tiltAngleMap = map(tiltAngle, 0, 480, 0, 180); // Adjust range if needed

    ServoController.servoWrite(baseServo, panAngleMap);
    ServoController.servoWrite(shoulderServo, tiltAngleMap);
}