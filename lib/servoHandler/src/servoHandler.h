#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include <vector>
#include <Adafruit_ADS1X15.h>


#define MAX_DOF 3
#define MAX_SERVO 4
 
constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint16_t MAX_ANGLE = 180;
constexpr uint8_t STARTAUTOMODE = -1;
constexpr uint8_t STOPAUTOMODE = -2;
constexpr bool OPEN_CLAW = true;
constexpr bool CLOSE_CLAW = false;

struct servoData
{
    uint16_t theta_1;
    uint16_t theta_2;
    uint16_t theta_3;
    bool claw = true;
    
    explicit servoData(int dataFromSerialPort[MAX_DOF]);
    std::vector<uint16_t> servoDataGet() const;
};

class servoHandler {
public:
    servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
    uint8_t elbowServoPin, uint8_t clawServoPin);

    void setForwardKinematic(double duration);
    void servoControllerBegin();
private:
    void getFromSerialPort();
    void driveServo(int startAngle = 0, int endAngle = 180, int delayTime = 15, int step = 1);

    static double forwardKinematicBase(double theta_s, int theta_f, double tf);
    static double forwardKinematicShoulder(double theta_s, int theta_f, double tf);
    static double forwardKinematicElbow(double theta_s, int theta_f, double tf);
    void autoModeHandler();
    void sendToServo(int theta_1, int theta_2, int theta_3, bool clawFlag);
    void clawFlagCheck(bool clawFlag);

    //servoData mServoData;
    iarduino_MultiServo SercoController;
    Adafruit_ADS1115 currentController;


    double previousAngles[MAX_DOF] = {0,0,0};
    int angleForwardKinematic[MAX_DOF + 1] = {90,90,90,90};
    int counterForServo = 0; 
    std::vector<servoData> autoModeAngles;  
    int numberOfElements{};
    int durationPerStep{};

    uint8_t baseServo;
    uint8_t shoulderServo;
    uint8_t elbowServo;
    uint8_t clawServo;
};



#endif //SERVOHANDLER_H
