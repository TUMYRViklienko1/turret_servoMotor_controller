#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <math.h>
#include <ServoDriverSmooth.h>

constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint8_t MAX_SERVO = 2;
constexpr uint8_t HEAD_DEGREE = 2;
constexpr uint16_t PWM_RESOLUTION = 4096;
constexpr uint16_t SERVO_PERIOD_US = 20000;

constexpr uint8_t BASESERVO = 0;     // Pin for pan
constexpr uint8_t SHOULDERSERVO = 1; // Pin for tilt

uint32_t myTimer;

ServoDriverSmooth ServoController[MAX_SERVO];
uint16_t headCoordinates[HEAD_DEGREE] = {INITIAL_ANGLE, INITIAL_ANGLE};

void getFromSerialPort();
void sendToServo();
void convertToPWM(uint16_t* dataFromSerial);

void setup() {
    Serial.begin(115200);

    while (!Serial);

    // Initialize servo settings
    ServoController[BASESERVO].attach(BASESERVO, SERVO_MIN, SERVO_MAX,105);
    ServoController[SHOULDERSERVO].attach(SHOULDERSERVO, SERVO_MIN, SERVO_MAX,130);


    ServoController[BASESERVO].smoothStart();
    ServoController[SHOULDERSERVO].smoothStart();
    
    //ser speed limit
    ServoController[BASESERVO].setSpeed(500);
    ServoController[BASESERVO].setAccel(1500);
    
    ServoController[SHOULDERSERVO].setSpeed(500);
    ServoController[SHOULDERSERVO].setAccel(1500);

}

void loop() {

    ServoController[BASESERVO].tick();
    ServoController[SHOULDERSERVO].tick();

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
    ServoController[BASESERVO].setTarget(panAngle);
    ServoController[SHOULDERSERVO].setTarget(tiltAngle);


}
