#include "servoHandler.h"
#define DURATION 1

#define xGAIN GAIN_ONE
#define x1BIT 0.125


int16_t ADCres; 
float Voltage = 0.0; 
bool flagClaw = true;

float curr = 0.0; 
#define zeroamp 2.482
#define oneamp 2.612

servoData::servoData(int dataFromSerialPort[4])
    : theta_1(dataFromSerialPort[0]), theta_2(dataFromSerialPort[1]), theta_3(dataFromSerialPort[2]), claw(dataFromSerialPort[3]) {}

 std::vector<uint16_t> servoData::servoDataGet() const{
  std::vector<uint16_t> temp =  {theta_1,theta_2,theta_3,claw};
  return temp;
}

servoHandler::servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin, uint8_t elbowServoPin,
uint8_t clawServoPin): baseServo(baseServoPin), shoulderServo(shoulderServoPin),elbowServo(elbowServoPin),
clawServo(clawServoPin) {}

void servoHandler::servoControllerBegin() {
  SercoController.servoSet(baseServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 0
  SercoController.servoSet(shoulderServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 1
  SercoController.servoSet(elbowServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 2
  SercoController.servoSet(clawServo,MAX_ANGLE, SERVO_MIN, SERVO_MAX);

  currentController.begin();
  currentController.setGain(xGAIN);
  SercoController.begin();

  SercoController.servoWrite(clawServo,0);

  //sendToServo(90,90,90,true);
}

void servoHandler::setForwardKinematic(const double duration) {
  static int durationPerStep;
  static int numberOfElements;
  getFromSerialPort(); // Get the angles from serial input

  if (angleForwardKinematic[0] == -1) {
    durationPerStep = angleForwardKinematic[1];
    numberOfElements = angleForwardKinematic[2];
   // sendToServo(40,40,40,true);
    return;
  }

  if (numberOfElements > 0) {
    const servoData temp(angleForwardKinematic);
    autoModeAngles.push_back(temp);
    numberOfElements--;
    if (numberOfElements == 0) {
      sendToServo(40,40,40,true);
      autoModeHandler();
      return;
    }
    return;
  }


  if (!autoModeAngles.empty())

  sendToServo(angleForwardKinematic[0], angleForwardKinematic[1], angleForwardKinematic[2], angleForwardKinematic[3]);
}


void servoHandler::getFromSerialPort() {
  if (Serial.available())
  {
    String dataFromSerialPort = Serial.readStringUntil('\n');
    int valueIndex = 0; // Index for angleForwardKinematic array
    int startIdx = 0; // Start index for substring
    int commaIdx;

    // Loop to parse the string
    while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1) {
      // Extract the substring and convert to integer
      angleForwardKinematic[valueIndex] = dataFromSerialPort.substring(startIdx, commaIdx).toInt();
      startIdx = commaIdx + 1; // Move to the next character after the comma
      valueIndex++;
    }

    // Handle the last value (after the final comma)
    angleForwardKinematic[valueIndex] = dataFromSerialPort.substring(startIdx).toInt();

  }
}




void servoHandler::autoModeHandler() {
  if (autoModeAngles.empty()) return; // Guard against empty vector

    for (int i = 0; i < autoModeAngles.size(); ++i) {
      sendToServo(autoModeAngles.at(i).theta_1,autoModeAngles.at(i).theta_2,autoModeAngles.at(i).theta_3,autoModeAngles.at(i).claw);
      delay(durationPerStep * 1000); // Blocking delay, adjust if asynchronous behavior is needed
    }
}


void servoHandler::sendToServo(int theta_1, int theta_2, int theta_3, bool claw) {
  if (counterForServo == 0) {
    for (double & previousAngle : previousAngles) {
      previousAngle = 90;
    }
  }

  // Interpolate angles over the specified duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (DURATION * 1000); // Convert duration to milliseconds

  while (millis() < endTime) {
    SercoController.servoWrite(baseServo,forwardKinematicBase(previousAngles[0], theta_1, DURATION));
    SercoController.servoWrite(shoulderServo,forwardKinematicShoulder(previousAngles[1], theta_2, DURATION));
    SercoController.servoWrite(elbowServo,forwardKinematicElbow(previousAngles[2], theta_3, DURATION));
    clawFlagCheck(claw);
    

    delay(10);
  }

  // Store previous angles for next loop iteration
  for (int i = 0; i < MAX_DOF; ++i) {
    previousAngles[i] = angleForwardKinematic[i];
  }

  counterForServo++;
}

void servoHandler::driveServo(int startAngle, int endAngle, int delayTime, int step) {
  for (int angle = startAngle; angle <= endAngle; angle += step) {
    SercoController.servoWrite(3,angle);
    Serial.print(angle);
    if (angle == 174) {
      flagClaw = false;
    }
    delay(delayTime);          // Wait for the servo to move to the position
  }
}

void servoHandler::clawFlagCheck(bool clawFlag) {

  if (clawFlag == OPEN_CLAW) {
    SercoController.servoWrite(3,0);
    flagClaw = true;
  } else if (clawFlag == CLOSE_CLAW) {
    if (flagClaw == true) {
      driveServo();
    }

    ADCres = currentController.readADC_SingleEnded(0);
    Voltage = (ADCres * x1BIT)/1000;
    curr = (Voltage - zeroamp) / (oneamp - zeroamp);

    if (curr > 0.6) {
      flagClaw = false;
    }

  }
}


double servoHandler::forwardKinematicBase(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // Final angle

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
    thet = theta_f;  // Snap to final angle after duration
  }

  return thet;
}

double servoHandler::forwardKinematicShoulder(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // Final angle

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
    thet = theta_f;  // Snap to final angle after duration
  }

  return thet;
}

double servoHandler::forwardKinematicElbow(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // Final angle

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
    thet = theta_f;  // Snap to final angle after duration
  }

  return thet;
}




