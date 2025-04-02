#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <math.h>


constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = (SERVO_MIN + SERVO_MAX)/2;
constexpr uint16_t MAX_ANGLE = 180;
constexpr uint8_t MAX_SERVO = 2;
constexpr uint8_t HEAD_DEGREE = 2;
constexpr uint16_t PWM_RESOLUTION = 4096;
constexpr uint16_t SERVO_PERIOD_US = 20000;

uint8_t baseServo = 0;     // Pin for pan
uint8_t shoulderServo = 1; // Pin for tilt

iarduino_MultiServo ServoController;
uint16_t headCoordinates[HEAD_DEGREE] = {INITIAL_ANGLE, INITIAL_ANGLE};

void getFromSerialPort();
void sendToServo();
void convertToPWM(uint16_t* dataFromSerial);

void setup() {
    Serial.begin(9600);

    while (!Serial);

    // Initialize servo settings
    ServoController.servoSet(baseServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    ServoController.servoSet(shoulderServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX);
    ServoController.begin();
    
    // Move servos to initial position
    ServoController.analogWrite(baseServo, INITIAL_ANGLE);
    ServoController.analogWrite(shoulderServo, INITIAL_ANGLE);
}

void loop() {
    
    if (Serial.available()) {
        
        getFromSerialPort();
        sendToServo();
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

void sendToServo() {
    Serial.println(headCoordinates[0]);
    int panAngle = (long(headCoordinates[0]) * PWM_RESOLUTION) / SERVO_PERIOD_US;
    int tiltAngle = (long(headCoordinates[1]) * PWM_RESOLUTION) / SERVO_PERIOD_US;
    
    Serial.println(panAngle);
    ServoController.analogWrite(baseServo, panAngle);
    ServoController.analogWrite(shoulderServo, tiltAngle);
}
