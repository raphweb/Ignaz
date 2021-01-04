#include <Servo.h>
#include <ESP32_ISR_Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <array>
#include <vector>

#define PWM_SERVOS 16
#define GPIO_SERVOS 4
#define SERVOS (PWM_SERVOS + GPIO_SERVOS)

#define BASEDELAYTIME 10

#define MAX_PULSE_LENGTH 2450.0
#define MIN_PULSE_LENGTH  550.0
#define MIN_ANGLE 0
#define MAX_ANGLE 180

#define ROBO_NAME "ignaz"
#define SERVO_DELAY "ServoDelay"
//#define SAVE_WIFI_CREDENTIALS

Preferences preferences;
WebServer server(80);

// Motion Data Index
int servoProgram;
int servoProgramStack;

typedef struct {
  const uint8_t pin;
  const char* name;
} servo_t;

const servo_t servos[SERVOS] = {
  {21, "Servo 0" }, {19, "Servo 1" }, {18, "Servo 2" },
  { 5, "Servo 3" }, {17, "Servo 4" }, {16, "Servo 5" },
  { 4, "Servo 6" }, {12, "Servo 7" }, {14, "Servo 8" },
  {27, "Servo 9" }, {26, "Servo 10"}, {25, "Servo 11"},
  {33, "Servo 12"}, {32, "Servo 13"}, {22, "Servo 14"},
  {23, "Servo 15"}, {13, "Servo 16"}, { 3, "Servo 17"},
  { 0, "Servo 18"}, { 1, "Servo 19"}
};

int8_t servoAdjustment[SERVOS] = {0};
uint32_t servoDelay = 0;

Servo servo[PWM_SERVOS];
uint8_t isrServo[GPIO_SERVOS];

uint8_t servoPosition[SERVOS];

typedef struct {
  const uint8_t index;
  const uint8_t angle;
} servo_angle_t;

typedef struct {
  const uint8_t moveTime;
  const std::vector<servo_angle_t> servoAngles;
} servo_positions_t;

const uint8_t zeroPosition[SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5   6   7   8    9  10   11  12   13  14   15  16  17  18  19 ms
  70, 90, 60, 120, 60, 145, 60, 90, 90, 120, 35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 0
};

const std::vector<servo_angle_t> idlePosition PROGMEM = {{
{0,  70}, {1,   90}, {2,  60}, {3,  120}, {4,  60}, {5,  145}, {6, 145}, {7,  90}, {8,  90}, {9,  35},
{10, 35}, {11, 120}, {12, 60}, {13, 120}, {14, 90}, {15, 110}, {16, 90}, {17, 90}, {18, 90}, {19, 90}
}};

const std::array<const servo_positions_t, 14> moveForward PROGMEM = {{
// transition from idle
{ 20, {{0, 60}, {6, 130}, {7, 80}, {8, 100}, {9, 50}, {11, 110}, {12, 90}, {13, 60}, {14, 60}, {15, 100}} },
// first pair of steps
{ 35, {                                                 {5, 131}, {10, 15},            {12, 100}, {13,  80}, {14, 80}} },
{ 25, {{0,  80}, {1, 110}, {2,  80}, {3, 115}, {4, 70}, {5, 117}, {10, 20}, {11, 125}, {12,  80}, {13, 110}, {14, 90}, {15, 120}} },
{ 20, {          {1, 120}, {2, 120}, {3,  90},          {5, 145}, {10, 35},            {12,  60}, {13, 120}} },
{ 35, {          {1, 100}, {2, 100}, {3,  80},          {5, 164}, {10, 50}} },
{ 25, {{0,  60}, {1,  90}, {2,  70}, {3, 100}, {4, 55}, {5, 160}, {10, 65}, {11, 110}, {12,  65}, {13, 100}, {14, 70}, {15, 100}} },
{ 25, {                    {2,  60}, {3, 120},          {5, 145}, {10, 35},            {12,  90}, {13,  60}, {14, 60}} },
// second pair of steps
{ 35, {                                                 {5, 131}, {10, 15},            {12, 100}, {13,  80}, {14, 80}} },
{ 25, {{0,  80}, {1, 110}, {2,  80}, {3, 115}, {4, 70}, {5, 117}, {10, 20}, {11, 125}, {12,  80}, {13, 110}, {14, 90}, {15, 120}} },
{ 20, {          {1, 120}, {2, 120}, {3,  90},          {5, 145}, {10, 35},            {12,  60}, {13, 120}} },
{ 35, {          {1, 100}, {2, 100}, {3,  80},          {5, 164}, {10, 50}} },
{ 25, {{0,  60}, {1,  90}, {2,  70}, {3, 100}, {4, 55}, {5, 160}, {10, 65}, {11, 110}, {12,  65}, {13, 100}, {14, 70}, {15, 100}} },
{ 25, {                    {2,  60}, {3, 120},          {5, 145}, {10, 35},            {12,  90}, {13,  60}, {14, 60}} },
// transition back to idle
{ 35 }
}};

const std::array<const servo_positions_t, 4> bow PROGMEM = {{
{ 50, {{5, 165}, {6, 125}, {7, 50}, {8, 150}, {10, 115}} },
{ 50, {{1, 75}, {3, 70}, {12, 110}, {14, 105}} },
{ 100 },
{ 80 }
}};

const std::array<const servo_positions_t, 9> waving PROGMEM = {{
{ 50, {{8, 70}, {9, 85}, {10, 135}, {16, 60}} },
{ 30, {{8, 90}, {9, 35}} },
{ 30, {{8, 70}, {9, 85}} },
{ 30, {{8, 90}, {9, 35}} },
{ 50, {{5, 45}, {6, 95}, {7, 110}, {10, 35}, {16, 120}} },
{ 30, {{6, 125}, {7, 90}} },
{ 30, {{6, 95}, {7, 110}} },
{ 30, {{6, 125}, {7, 90}} },
{ 80 }
}};

const std::array<std::array<uint8_t, (SERVOS + 1)>, 7> apache PROGMEM = {{
// 0   1   2    3   4    5    6   7    8   9   10   11  12   13  14   15   16  17  18  19 10ms
{ 80, 90, 60, 120, 70, 145, 115, 45,  90, 95,  35, 125, 60, 120, 90, 120,  90, 90, 90, 90, 50 },
{ 80, 90, 60, 120, 70, 145, 115, 45,  90, 95,  35, 125, 60, 120, 90, 125,  90, 90, 90, 90, 30 },
{ 80, 90, 60, 120, 70, 145, 115, 45, 150, 25, 115, 110, 60, 120, 90, 130,  90, 90, 90, 90, 50 },
{ 80, 90, 60, 120, 70, 145, 115, 45, 150, 25, 115, 110, 60, 120, 90, 130, 120, 90, 90, 90, 80 },
{ 80, 90, 60, 120, 70, 145, 115, 45,  90, 95,  35, 125, 60, 120, 90, 125, 120, 90, 90, 90, 60 },
{ 80, 90, 60, 120, 70, 145, 115, 45,  90, 95,  35, 125, 60, 120, 90, 120,  90, 90, 90, 90, 50 },
{ 70, 90, 60, 120, 60, 145, 145, 90,  90, 35,  35, 120, 60, 120, 90, 110,  90, 90, 90, 90, 80 }
}};

const std::array<std::array<uint8_t, (SERVOS + 1)>, 12> balance PROGMEM = {{
// 0   1   2    3   4    5    6   7   8    9  10   11  12   13  14   15   16  17  18  19 10ms
{ 60, 90, 60, 120, 55, 145,  60, 90, 90, 120, 35, 110, 60, 120, 90, 100,  90, 90, 90, 90, 50 },
{ 60, 90, 60, 120, 55,  95,  60, 90, 90, 120, 85, 110, 60, 120, 95, 100,  90, 90, 90, 90, 30 },
{ 55, 90, 60, 120, 70,  95,  60, 90, 90, 120, 85, 110, 60, 120, 95, 100,  90, 90, 90, 90, 50 },
{ 55, 90, 60, 120, 70,  95,  60, 90, 90, 120, 85, 110, 60, 120, 95, 100,  90, 90, 90, 90, 60 },
{ 55, 70, 60,  60, 70,  95,  60, 90, 90, 120, 85, 110, 60, 120, 95, 100,  90, 90, 90, 90, 90 },
{ 55, 70, 60,  60, 70,  95,  40, 90, 90, 140, 85, 110, 60, 120, 95, 100,  60, 90, 90, 90, 50 },
{ 55, 70, 60,  60, 70,  95,  80, 90, 90, 100, 85, 110, 60, 120, 95, 100, 120, 90, 90, 90, 50 },
{ 55, 70, 60,  60, 70,  95,  40, 90, 90, 140, 85, 110, 60, 120, 95, 100,  60, 90, 90, 90, 50 },
{ 55, 70, 60,  60, 70,  95,  80, 90, 90, 100, 85, 110, 60, 120, 95, 100, 120, 90, 90, 90, 50 },
{ 55, 70, 60,  60, 70,  95,  60, 90, 90, 120, 85, 110, 60, 120, 95, 100,  90, 90, 90, 90, 50 },
{ 55, 90, 60, 120, 70,  95,  60, 90, 90, 120, 85, 110, 60, 120, 95, 100,  90, 90, 90, 90, 80 },
{ 70, 90, 60, 120, 60, 145, 145, 90, 90,  35, 35, 120, 60, 120, 90, 110,  90, 90, 90, 90, 80 }
}};

const std::array<std::array<uint8_t, (SERVOS + 1)>, 12> warmUp PROGMEM = {{
// 0    1    2    3   4    5    6   7    8    9   10   11   12   13  14   15   16  17  18  19 10ms
{ 70,  90,  60, 120, 60, 145, 105, 30, 150,  75,  35, 120,  60, 120, 90, 110,  90, 90, 90, 90, 80 },
{ 80,  90,  60, 120, 70, 145, 105, 30,  90, 125,  35, 125,  60, 120, 90, 120,  60, 90, 90, 90, 80 },
{ 80,  90,  60, 120, 70, 145, 105, 30,  90, 125,  35, 125,  60, 120, 90, 120,  60, 90, 90, 90, 50 },
{ 60,  90,  60, 120, 55, 145,  55, 90, 150,  75,  35, 110,  60, 120, 90, 100, 120, 90, 90, 90, 80 },
{ 60,  90,  60, 120, 55, 145,  55, 90, 150,  75,  35, 110,  60, 120, 90, 100, 120, 90, 90, 90, 50 },
{ 70,  90,  60, 120, 60, 145, 105, 30, 150,  75,  35, 120,  60, 120, 90, 110,  90, 90, 90, 90, 50 },
{ 70, 117, 125,  77, 60,  60, 145, 90,  90,  50, 115, 120, 100,  55, 63, 110,  90, 90, 90, 90, 80 },
{ 70, 117, 125,  77, 60,  60, 145, 90,  90,  50, 115, 120, 100,  55, 63, 110,  90, 90, 90, 90, 30 },
{ 70,  90,  60, 120, 60, 145, 105, 30, 150,  75,  35, 120,  60, 120, 90, 110,  90, 90, 90, 90, 50 },
{ 70, 117, 125,  77, 60,  60, 145, 90,  90,  50, 115, 120, 100,  55, 63, 110,  90, 90, 90, 90, 80 },
{ 70, 117, 125,  77, 60,  60, 145, 90,  90,  50, 115, 120, 100,  55, 63, 110,  90, 90, 90, 90, 30 },
{ 70,  90,  60, 120, 60, 145, 145, 90,  90,  35,  35, 120,  60, 120, 90, 110,  90, 90, 90, 90, 80 }
}};

const std::array<std::array<uint8_t, (SERVOS + 1)>, 10> clapHands PROGMEM = {{
// 0   1   2    3   4    5    6   7    8   9   10   11  12   13  14   15  16  17  18  19 10ms
{ 70, 90, 60, 120, 60, 145, 145, 90,  90, 35,  35, 120, 60, 120, 90, 110, 90, 90, 90, 90,  5 }, // 1
{ 70, 90, 60, 120, 60,  70, 145, 90,  90, 35, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 45 }, // 2
{ 70, 90, 60, 120, 60,  70, 145, 44, 136, 35, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 15 }, // 3
{ 70, 90, 60, 120, 60,  70, 165, 44, 136, 15, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 }, // 4
{ 70, 90, 60, 120, 60,  70, 145, 44, 136, 35, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 }, // 5
{ 70, 90, 60, 120, 60,  70, 165, 44, 136, 15, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 }, // 4
{ 70, 90, 60, 120, 60,  70, 145, 44, 136, 35, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 }, // 5
{ 70, 90, 60, 120, 60,  70, 165, 44, 136, 15, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 }, // 4
{ 70, 90, 60, 120, 60,  70, 145, 44, 136, 35, 110, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 }, // 5
{ 70, 90, 60, 120, 60, 145, 145, 90,  90, 35,  35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 45 }  // 1
}};

void setAngle(uint8_t servoIndex, uint8_t servoAngle) {
  if ((servoIndex & 16) == 0) {
    servo[servoIndex].write(servoAngle);
  } else {
    ESP32_ISR_Servos.setPosition(isrServo[servoIndex & 3], servoAngle);
  }
}

void servoProgramZero() {
  for(uint8_t i = 0; i < SERVOS; i++) {
    servoPosition[i] = zeroPosition[i] + servoAdjustment[i];
  }
  for(uint8_t iServo = 0; iServo < SERVOS; iServo++) {
    setAngle(iServo, servoPosition[iServo]);
    delay(BASEDELAYTIME);
  }
}

void servoProgramIdle() {
  for(const auto& servoAngle : idlePosition) {
    servoPosition[servoAngle.index] = servoAngle.angle + servoAdjustment[servoAngle.index];
  }
  for(uint8_t iServo = 0; iServo < SERVOS; iServo++) {
    setAngle(iServo, servoPosition[iServo]);
    delay(BASEDELAYTIME);
  }
}

template<std::size_t SIZE> void servoProgramRun(const std::array<const servo_positions_t, SIZE>& moveMatrix) {
  uint8_t targetPositions[SERVOS];
  for(uint8_t movInd = 0; movInd < SIZE; movInd++) {
    const uint16_t moveTime = moveMatrix[movInd].moveTime * BASEDELAYTIME + servoDelay;
    if (moveMatrix[movInd].servoAngles.empty() && movInd < SIZE - 1) {
      // just delay, no servo movements
      delay(moveTime);
    } else {
      const std::vector<servo_angle_t>& targetServoPositions =
        (movInd == SIZE - 1) ?
          idlePosition :   // for last servo positions go back to idle
          moveMatrix[movInd].servoAngles;
      // calculate target positions for use in the nested second loop
      for(const auto& servoAngle : targetServoPositions) {
        targetPositions[servoAngle.index] = servoAngle.angle + servoAdjustment[servoAngle.index];
      }
      const uint8_t stepCount = moveTime / BASEDELAYTIME;
      for(uint8_t step = 0; step < stepCount; step++) {
        for(const auto& servoAngle : targetServoPositions) {
          const uint8_t originPosition = servoPosition[servoAngle.index];
          const uint8_t targetPosition = targetPositions[servoAngle.index];
          // absolute step size by which the angle shall be updated
          const uint8_t stepDifference = map(BASEDELAYTIME * step, 0, moveTime, 0,
            (originPosition > targetPosition ?
              originPosition - targetPosition: targetPosition - originPosition));
          // new angle to be set for this step
          const uint8_t stepAngle = (originPosition > targetPosition ?
            originPosition - stepDifference : originPosition + stepDifference);
          // only set the step angle if we have not reached the target position yet
          if ((originPosition <= targetPosition && stepAngle <= targetPosition) ||
              (originPosition >  targetPosition && stepAngle >= targetPosition)) {
            setAngle(servoAngle.index, stepAngle);
          }
        }
        delay(BASEDELAYTIME);
      }
    }
    for(const auto& servoAngle : moveMatrix[movInd].servoAngles) {
      servoPosition[servoAngle.index] = servoAngle.angle + servoAdjustment[servoAngle.index];
    }
  }
}

template<std::size_t SIZE> void servoProgramRun(const std::array<std::array<uint8_t, SERVOS + 1>, SIZE>& iMatrix) {
  int tmpa, tmpb, tmpc;
  for(uint8_t mainLoopIndex = 0; mainLoopIndex < SIZE; mainLoopIndex++) {
    const int interTotalTime = iMatrix[mainLoopIndex][SERVOS] * BASEDELAYTIME + servoDelay;
    const int interDelayCounter = interTotalTime / BASEDELAYTIME;
    for(uint8_t interStepLoop = 0; interStepLoop < interDelayCounter; interStepLoop++) {
      for(uint8_t servoIndex = 0; servoIndex < SERVOS; servoIndex++) {
        tmpa = servoPosition[servoIndex];
        tmpb = iMatrix[mainLoopIndex][servoIndex] + servoAdjustment[servoIndex];
        if (tmpa == tmpb) {
          tmpc = tmpb;
        } else if (tmpa > tmpb) {
          tmpc =  map(BASEDELAYTIME * interStepLoop, 0, interTotalTime, 0, tmpa - tmpb);
          if (tmpa - tmpc >= tmpb) {
            setAngle(servoIndex, tmpa - tmpc);
          }
        } else if (tmpa < tmpb) {
          tmpc =  map(BASEDELAYTIME * interStepLoop, 0, interTotalTime, 0, tmpb - tmpa);
          if (tmpa + tmpc <= tmpb) {
            setAngle(servoIndex, tmpa + tmpc);
          }
        }
      }
      delay(BASEDELAYTIME);
    }
    for(uint8_t i = 0; i < SERVOS; i++) {
      servoPosition[i] = iMatrix[mainLoopIndex][i] + servoAdjustment[i];
    }
  }
}

void enableServos() {
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].attach(servos[i].pin, i, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  }
  for(uint8_t i = PWM_SERVOS; i < SERVOS; i++) {
    ESP32_ISR_Servos.enable(isrServo[i & 3]);
  }
}

void disableServos() {
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].detach();
  }
  for(uint8_t i = PWM_SERVOS; i < SERVOS; i++) {
    ESP32_ISR_Servos.disable(isrServo[i & 3]);
  }
}

void handleSave() {
  String key = server.arg("key");
  String value = server.arg("value");
  const uint8_t keyInt = key.toInt();
  
  delay(50);
  if (keyInt == 100) {
    for(uint8_t i = 0; i < SERVOS; i++) {
      preferences.putChar(servos[i].name, 0);
      servoAdjustment[i] = 0;
    }
    preferences.putChar(SERVO_DELAY, 0);
    servoDelay = 0;
  } else if (keyInt < SERVOS) {
    const int8_t valueInt = value.toInt();
    if (valueInt >= -125 && valueInt <= 125) {
      preferences.putChar(servos[keyInt].name, valueInt);
      servoAdjustment[keyInt] = valueInt;
    }
  } else if (keyInt == SERVOS) {
    const uint32_t valueUInt = value.toInt();
    preferences.putUInt(SERVO_DELAY, valueUInt);
    servoDelay = valueUInt;
  }
  delay(50);
  
  server.send(200, "text/html", "(key, value)=(" + key + "," + value + ")");
}

void handleController() {
  String pm = server.arg("pm");
  String pms = server.arg("pms");
  String servoName = server.arg("servo");

  if (pm != "") {
    servoProgram = pm.toInt();
  }
  if (pms != "") {
    servoProgramStack = pms.toInt();
  }

  if (servoName != "") {
    String ival = server.arg("value");
    const uint8_t servoId = servoName.toInt();
    const uint8_t servoAngle = ival.toInt() + servoAdjustment[servoId];
    setAngle(servoId, servoAngle);
  }

  server.send(200, "text/html", "(pm, pms)=(" + pm + "," + pms + ")");
}

void handleGetPreferences() {
  String content = "";
  for(uint8_t i = 0; i < SERVOS - 1; i++) {
    content += String(servoAdjustment[i]) + ", ";
  }
  content += String(servoAdjustment[SERVOS - 1]);
  server.send(200, "text/html", content);
}

void handleZero() {
  const String htmlbackground[SERVOS/2] = {"3498db", "bdbdbd", "f5da81", "bdbdbd", "3498db", "3498db", "bdbdbd", "f5da81", "6e6e6e", "6e6e6e"};
  
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "<title>Ignaz Zero Check</title>";
  content += " <meta charset=\"UTF-8\">";
  content += " <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 160px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += "  font-size: 24px;";
  content += "  background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";
  content += ".pm_text {";
  content += "width: 160px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "font-size: 24px;";

  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";
  content += "  </style>";
  content += "</head>";
  content += "<body>";

  content += "<table align=center>";
  for(uint8_t i = SERVOS / 2; i > 0; i--) {
    const uint8_t  firstServoNo = (i > PWM_SERVOS / 2) ? ((i*2)-1) : (PWM_SERVOS-i);
    const uint8_t secondServoNo = (i > PWM_SERVOS / 2) ? ((i-1)*2) : (i-1);
    const String  firstServoStr = String(firstServoNo);
    const String secondServoStr = String(secondServoNo);
    const String  background = htmlbackground[i-1];
    content += "<tr>";
    content += "<td><button class=\"pm_btn\" style=\"background: #" + htmlbackground[i-1] + ";\" type=\"button\" onclick=\"controlServo(" + firstServoStr +
      ", " + String(zeroPosition[firstServoNo]) + ")\">" + servos[firstServoNo].name + "</button></td>";
    content += "<td><button class=\"pm_btn\" style=\"background: #" + htmlbackground[i-1] + ";\" type=\"button\" onclick=\"controlServo(" + secondServoStr +
      ", " + String(zeroPosition[secondServoNo]) + ")\">" + servos[secondServoNo].name + "</button></td>";
    content += "</tr>";
    if (i == 9 || i == 6 || i == 1) {
      content += "</table>";
      content += "<br>";
      content += "<table align=center>";
    }
  }
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(100)\">ALL</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "</body>";
  content += "<script>";
  content += "function controlServo(id, value) {";
  content += "  var xhttp = new XMLHttpRequest();";

  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?servo=\"+id+\"&value=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "function controlPm(value) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pm=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

void handleSetting() {
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "  <title>Ignaz Setting</title>";
  content += "  <meta charset=\"UTF-8\">";
  content += "  <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 120px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += "  font-size: 24px;";
  content += "  background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";
  content += ".pm_text {";
  content += "width: 100px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "font-size: 24px;";

  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";
  content += "  </style>";
  content += "</head>";
  content += "<body>";

  content += "<table align=center>";
  for(uint8_t i = SERVOS / 2; i > 0; i--) {
    const uint8_t  firstServoNum = (i > PWM_SERVOS / 2) ? ((i*2)-1) : (PWM_SERVOS-i);
    const uint8_t secondServoNum = (i > PWM_SERVOS / 2) ? ((i-1)*2) : (i-1);
    const String  firstServoStr = String(firstServoNum );
    const String secondServoStr = String(secondServoNum);
    content += "<tr>";
    content += "<td>Servo " + firstServoStr + "<br/><input class=\"pm_text\" type=\"text\" id=\"servo_" + firstServoStr + "\" value=\"" + String(servoAdjustment[firstServoNum]) +
      "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(" + firstServoStr + ",'servo_" + firstServoStr + "')\">SET</button></td>";
    content += "<td>Servo " + secondServoStr + "<br/><input class=\"pm_text\" type=\"text\" id=\"servo_" + secondServoStr + "\" value=\"" + String(servoAdjustment[secondServoNum]) +
      "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(" + secondServoStr + ",'servo_" + secondServoStr + "')\">SET</button></td>";
    content += "</tr>";
  }
  content += "</table>";

  content += "<br>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td>Delay Time<br/><input class=\"pm_text\" type=\"text\" id=\"servo_delay\" value=\"" + String(servoDelay) +
    "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(20,'servo_delay')\">SET</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";
  content += "<br>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"saveServo(100, 0)\">RESET</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "</body>";
  content += "<script>";

  content += "function saveServo(id, textId) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  var value = \"0\";";
  content += "  if(id==100){";
  for(uint8_t i = 0; i < SERVOS; i++) {
    content += "    document.getElementById(\"servo_" + String(i) + "\").value = \"0\";";
  }
  content += "    document.getElementById(\"servo_delay\").value = \"0\";";
  content += "  }else{";
  content += "    value = document.getElementById(textId).value;";
  content += "  }";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "     document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\",\"save?key=\"+id+\"&value=\"+value, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "</script>";
  content += "</html>";
  server.send(200, "text/html", content);
}


void handleIndex() {
  String content = "";
  content += "<html>";

  content += "<head>";
  content += "  <title>Ignaz Controller</title>";
  content += "  <meta charset=\"UTF-8\">";
  content += "  <style type=\"text/css\">";
  content += "  body {";
  content += "    color: white;";
  content += "    background-color: #000000 }";

  content += "  .pm_btn {";
  content += "  width: 160px;";
  content += "  -webkit-border-radius: 5;";
  content += "  -moz-border-radius: 5;";
  content += "  border-radius: 5px;";
  content += "  font-family: Arial;";
  content += "  color: #ffffff;";
  content += " font-size: 24px;";
  content += "background: #3498db;";
  content += "  padding: 10px 20px 10px 20px;";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pm_btn:hover {";
  content += "  background: #3cb0fd;";
  content += "  background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "  background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "  text-decoration: none;";
  content += "}";

  content += ".pms_btn {";
  content += "width: 240px;";
  content += "-webkit-border-radius: 5;";
  content += "-moz-border-radius: 5;";
  content += "border-radius: 5px;";
  content += "font-family: Arial;";
  content += "color: #ffffff;";
  content += "font-size: 24px;";
  content += "background: #3498db;";
  content += "padding: 10px 20px 10px 20px;";
  content += "text-decoration: none;";
  content += "}";

  content += ".pms_btn:hover {";
  content += "background: #3cb0fd;";
  content += "background-image: -webkit-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -moz-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -ms-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: -o-linear-gradient(top, #3cb0fd, #3498db);";
  content += "background-image: linear-gradient(to bottom, #3cb0fd, #3498db);";
  content += "text-decoration: none;";
  content += "}";
  content += " </style>";
  content += "</head>";

  content += "<body>";
  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(3)\">TurnLeft</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(1)\">Vorw&auml;rts</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(4)\">TurnRight</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(5)\">MoveLeft</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(99)\">RUHEN</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(6)\">MoveRight</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(7)\">LeftKick</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(2)\">Backward</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(8)\">RightKick</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" type=\"button\" onclick=\"controlPm(9)\">SkatingForward</button></td>";
  content += "<td><button class=\"pms_btn\" type=\"button\" onclick=\"controlPm(10)\">SkatingBackward</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pms_btn\" type=\"button\" onclick=\"controlPm(11)\">GetUp</button></td>";
  content += "<td><button class=\"pms_btn\" type=\"button\" onclick=\"controlPm(12)\">FaceDownGetUp</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(1)\">Verbeugen</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(2)\">Winken</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(7)\">Klatschen</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(3)\">Iron Man</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(4)\">Apache</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(5)\">Balancieren</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(6)\">Aufw&auml;rmen</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(8)\">Dance</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(9)\">Push-Up</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(10)\">TaiChi</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td colspan=\"2\"><center><button  class=\"pms_btn\" style=\"background: #04b404;\" type=\"button\" onclick=\"controlPms(99)\">Auto Demo</button></center></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table align=center><tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #121314;\" type=\"button\" onclick=\"controlPm(97)\">Servos Aus</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #121314;\" type=\"button\" onclick=\"controlPm(98)\">Servos An</button></td>";
  content += "</tr></table>";

  content += "</body>";
  content += "<script>";
  content += "function controlPm(id) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";

  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pm=\"+id, true);";
  content += "  xhttp.send();";
  content += "}";
  content += "function controlPms(id) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (xhttp.readyState == 4 && xhttp.status == 200) {";

  content += " }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pms=\"+id, true);";
  content += "  xhttp.send();";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  preferences.begin(ROBO_NAME);

#ifdef SAVE_WIFI_CREDENTIALS
  const char* ssid = "WLAN";
  const char* password = "****";
  preferences.clear();
  preferences.putString("SSID", ssid);
  preferences.putString("PASS", password);
#else
  String ssidStr = preferences.getString("SSID");
  const char* ssid = ssidStr.c_str();
  String passwordStr = preferences.getString("PASS");
  const char* password = passwordStr.c_str();
  Serial.println("Loaded SSID: " + ssidStr + ", PASS: " + passwordStr);
#endif
  
  // Connect to Wi-Fi network with SSID and password
  Serial.println("Connecting to " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setHostname(ROBO_NAME);
  WiFi.setAutoReconnect(true);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi connection failed!");
    delay(1000);
    ESP.restart();
  }
  // Print local IP address and start web server
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/controller", HTTP_GET, handleController);
  server.on("/save", HTTP_GET, handleSave);

  server.on("/", HTTP_GET, handleIndex);
  //server.on("/editor", HTTP_GET, handleEditor);
  server.on("/zero", HTTP_GET, handleZero);
  server.on("/setting", HTTP_GET, handleSetting);

  //server.on("/online", HTTP_GET, handleOnLine);
  server.on("/prefs", HTTP_GET, handleGetPreferences);

  server.begin();
  Serial.println("HTTP server started");

  // load preferences and idle position
  for(uint8_t i = 0; i < SERVOS; i++) {
    servoAdjustment[i] = preferences.getChar(servos[i].name);
    servoPosition[i] = idlePosition[i].angle + servoAdjustment[i];
  }
  servoDelay = preferences.getUInt(SERVO_DELAY);

  // init servos
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].attach(servos[i].pin, i, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    setAngle(i, servoPosition[i]);
    delay(60);
  }
  for(uint8_t i = PWM_SERVOS; i < SERVOS; i++) {
    isrServo[i & 3] = ESP32_ISR_Servos.setupServo(servos[i].pin, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    setAngle(i, servoPosition[i]);
    delay(60);
  }
}

void loop() {
  server.handleClient();

  if (servoProgram >= 1) {
    Serial.println("servoProgram = " + String(servoProgram));

    switch (servoProgram) {
      case 1:
        servoProgramRun(moveForward);
        servoProgramIdle();
        break;
      case 97:
        disableServos();
        break;
      case 98:
        enableServos();
        servoProgramIdle();
        delay(300);
        break;
      case 99:
        servoProgramIdle();
        delay(300);
        break;
      case 100:
        servoProgramZero();
        delay(300);
        break;
    }
    servoProgram = 0;
  }

  if (servoProgramStack >= 1) {
    Serial.println("servoProgramStack = " + String(servoProgramStack));

    switch (servoProgramStack) {
      case 1:
        servoProgramRun(bow);
        servoProgramIdle();
        break;
      case 2:
        servoProgramRun(waving);
        servoProgramIdle();
        break;
      case 4:
        servoProgramRun(apache);
        servoProgramIdle();
        break;
      case 5:
        servoProgramRun(balance);
        servoProgramIdle();
        break;
      case 6:
        servoProgramRun(warmUp);
        servoProgramIdle();
        break;
      case 7:
        servoProgramRun(clapHands);
        servoProgramIdle();
        break;
      case 99:
        servoProgramRun(waving);
        servoProgramIdle();
        delay(500);
        servoProgramRun(warmUp);
        servoProgramIdle();
        delay(500);
        servoProgramRun(apache);
        servoProgramIdle();
        delay(500);
        servoProgramRun(balance);
        servoProgramIdle();
        delay(500);
        servoProgramRun(clapHands);
        servoProgramIdle();
        delay(500);
        servoProgramRun(bow);
        servoProgramIdle();
        break;
    }
    servoProgramStack = 0;
  }
}
