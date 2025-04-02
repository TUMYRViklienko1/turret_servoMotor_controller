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
        Serial.write("1");
        delay(10);
        //getFromSerialPort();
        sendToServo();
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
        headCoordinates[valueIndex] =dataFromSerialPort.substring(startIdx).toInt();
    }
   

  }

void sendToServo() {
    auto panAngle = int((headCoordinates[0] * PWM_RESOLUTION) / SERVO_PERIOD_US);
    auto tiltAngle = int((headCoordinates[1] * PWM_RESOLUTION) / SERVO_PERIOD_US);


    ServoController.analogWrite(baseServo, panAngle);
    ServoController.analogWrite(shoulderServo, tiltAngle);
}
