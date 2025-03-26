#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <math.h>
#include <GyverPID.h>

constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint16_t MAX_ANGLE = 180;
#define MAX_SERVO 2
#define HEADDEGREE 2

uint8_t baseServo = 0;     // Pin for pan
uint8_t shoulderServo = 1; // Pin for tilt

iarduino_MultiServo SercoController;
uint16_t servoMotorAngles[MAX_SERVO]; // Array to hold angles
uint16_t headCoordinates[HEADDEGREE];

GyverPID pidPan(1.4,1.1,1.3,10);
GyverPID pidTilt(2.5,2.0,3.0,10);

void pidCountrol();

void sendToServo(const uint16_t panAngle, const uint16_t tiltAngle);

void getFromSerialPort();

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

    pidPan.setDirection(NORMAL);
    pidTilt.setDirection(NORMAL);
    pidPan.setpoint = 320;
    pidTilt.setpoint = 240;
}

void loop() {
    if (Serial.available()) {
        getFromSerialPort(); // Get coordinates from serial port
        pidCountrol();
    }
}

void getFromSerialPort() {
    String dataFromSerialPort = Serial.readStringUntil('\n');
    int valueIndex = 0; // Index for servoMotorAngles array
    int startIdx = 0;   // Start index for substring
    int commaIdx;
  
    // Loop to parse the string
    while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1 && valueIndex < MAX_SERVO) {
        headCoordinates[valueIndex] =  dataFromSerialPort.substring(startIdx, commaIdx).toInt();
        startIdx = commaIdx + 1;
        valueIndex++;
    }
  
    // Handle the last value
    if (valueIndex < MAX_SERVO) {
        headCoordinates[valueIndex] = dataFromSerialPort.substring(startIdx).toInt();
    }
  }


  void pidCountrol() {
    static uint32_t tmr;
    if (millis() - tmr > 10) {
        tmr = millis();

        pidPan.input = headCoordinates[0];
        uint16_t panAngle = pidPan.output;

        pidTilt.input = headCoordinates[1];
        uint16_t tiltAngle = pidTilt.output;

        Serial.print("DEBUG: PID Pan Output = "); Serial.println(pidPan.output);
        

        sendToServo(panAngle, tiltAngle);
    }
}


void sendToServo(const uint16_t panAngle, const uint16_t tiltAngle) {

    uint16_t panAngleMap = map(panAngle, 0, 640, 0, 180) ;
    //uint16_t tiltAngleMao = map(tiltAngle, 10, 240, 0, 180) + 90;

    SercoController.servoWrite(baseServo, panAngleMap);
    //SercoController.servoWrite(shoulderServo, tiltAngleMao);
delay(10);
}
