#include <Servo.h>
#include <ESP32_ISR_Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

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

const uint8_t zeroPosition[SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5   6   7   8    9  10   11  12   13  14   15  16  17  18  19 ms
  70, 90, 60, 120, 60, 145, 60, 90, 90, 120, 35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 0
};

const uint8_t idlePosition[SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5    6   7   8   9  10   11  12   13  14   15  16  17  18  19 ms
  70, 90, 60, 120, 60, 145, 145, 90, 90, 35, 35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 0
};

const uint8_t moveForwardSteps = 15;
uint8_t moveForward[moveForwardSteps][SERVOS + 1] PROGMEM = {
// 0    1    2    3   4    5    6   7    8   9  10   11   12   13  14   15  16  17  18  19 10ms
{ 70,  90,  60, 120, 60, 145, 145, 90,  90, 35, 35, 120,  60, 120, 90, 110, 90, 90, 90, 90, 50 }, //  1
{ 60,  90,  60, 120, 55, 145, 135, 80, 100, 45, 35, 109,  90,  60, 60, 100, 90, 90, 90, 90, 20 }, //  2
{ 60,  90,  60, 120, 57, 131, 130, 80, 100, 50, 16, 111, 100,  80, 80, 100, 90, 90, 90, 90, 35 }, //  3
{ 80, 110,  80, 115, 68, 117, 130, 80, 100, 50, 20, 123,  80, 110, 90, 120, 90, 90, 90, 90, 25 }, //  4
{ 80, 120, 120,  90, 69, 145, 130, 80, 100, 50, 35, 123,  60, 120, 90, 120, 90, 90, 90, 90, 20 }, //  5
{ 80, 100, 100,  80, 69, 164, 130, 80, 100, 50, 49, 123,  60, 120, 90, 120, 90, 90, 90, 90, 35 }, //  6
{ 60,  90,  70, 100, 57, 160, 130, 80, 100, 50, 63, 112,  65, 100, 70, 100, 90, 90, 90, 90, 25 }, //  7
{ 60,  90,  60, 120, 58, 145, 130, 80, 100, 50, 35, 111,  90,  60, 60, 100, 90, 90, 90, 90, 25 }, //  8
{ 60,  90,  60, 120, 57, 131, 130, 80, 100, 50, 16, 111, 100,  80, 80, 100, 90, 90, 90, 90, 35 }, //  3
{ 80, 110,  80, 115, 68, 117, 130, 80, 100, 50, 20, 123,  80, 110, 90, 120, 90, 90, 90, 90, 25 }, //  4
{ 80, 120, 120,  90, 69, 145, 130, 80, 100, 50, 35, 123,  60, 120, 90, 120, 90, 90, 90, 90, 20 }, //  5
{ 80, 100, 100,  80, 69, 164, 130, 80, 100, 50, 49, 123,  60, 120, 90, 120, 90, 90, 90, 90, 35 }, //  6
{ 60,  90,  70, 100, 57, 160, 130, 80, 100, 50, 63, 112,  65, 100, 70, 100, 90, 90, 90, 90, 25 }, //  7
{ 60,  90,  60, 120, 58, 145, 130, 80, 100, 50, 35, 111,  90,  60, 60, 100, 90, 90, 90, 90, 25 }, //  8
{ 70,  90,  60, 120, 60, 145, 145, 90,  90, 35, 35, 120,  60, 120, 90, 110, 90, 90, 90, 90, 50 }  //  1
};

const uint8_t wavingSteps = 9;
uint8_t waving[wavingSteps][SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5    6    7   8   9   10   11  12   13  14   15   16  17  18  19 10ms
{ 70, 90, 60, 120, 60, 145, 145,  90, 70, 85, 135, 120, 60, 120, 90, 110,  60, 90, 90, 90, 50 },
{ 70, 90, 60, 120, 60, 145, 145,  90, 90, 35, 135, 120, 60, 120, 90, 110,  60, 90, 90, 90, 30 },
{ 70, 90, 60, 120, 60, 145, 145,  90, 70, 85, 135, 120, 60, 120, 90, 110,  60, 90, 90, 90, 30 },
{ 70, 90, 60, 120, 60, 145, 145,  90, 90, 35, 135, 120, 60, 120, 90, 110,  60, 90, 90, 90, 30 },
{ 70, 90, 60, 120, 60,  45,  95, 110, 90, 35,  35, 120, 60, 120, 90, 110, 120, 90, 90, 90, 50 },
{ 70, 90, 60, 120, 60,  45, 125,  90, 90, 35,  35, 120, 60, 120, 90, 110, 120, 90, 90, 90, 30 },
{ 70, 90, 60, 120, 60,  45,  95, 110, 90, 35,  35, 120, 60, 120, 90, 110, 120, 90, 90, 90, 30 },
{ 70, 90, 60, 120, 60,  45, 125,  90, 90, 35,  35, 120, 60, 120, 90, 110, 120, 90, 90, 90, 30 },
{ 70, 90, 60, 120, 60, 145, 145,  90, 90, 35,  35, 120, 60, 120, 90, 110,  90, 90, 90, 90, 80 }
};

const uint8_t clapHandsSteps = 10;
uint8_t clapHands[clapHandsSteps][SERVOS + 1] PROGMEM = {
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
};

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
  for(uint8_t i = 0; i < SERVOS; i++) {
    servoPosition[i] = idlePosition[i] + servoAdjustment[i];
  }
  for(uint8_t iServo = 0; iServo < SERVOS; iServo++) {
    setAngle(iServo, servoPosition[iServo]);
    delay(BASEDELAYTIME);
  }
}

void servoProgramRun(uint8_t iMatrix[][SERVOS + 1], uint8_t iSteps) {
  int tmpa, tmpb, tmpc;
  for(uint8_t mainLoopIndex = 0; mainLoopIndex < iSteps; mainLoopIndex++) {
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
  uint8_t keyInt = key.toInt();
  
  delay(50);
  if (keyInt == 100) {
    for(uint8_t i = 0; i < SERVOS; i++) {
      preferences.putChar(servos[i].name, 0);
      servoAdjustment[i] = 0;
    }
    preferences.putChar(SERVO_DELAY, 0);
    servoDelay = 0;
  } else if (keyInt < SERVOS) {
    int8_t valueInt = value.toInt();
    if (valueInt >= -125 && valueInt <= 125) {
      preferences.putChar(servos[keyInt].name, valueInt);
      servoAdjustment[keyInt] = valueInt;
    }
  } else if (keyInt == SERVOS) {
    uint32_t valueUInt = value.toInt();
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
    uint8_t servoId = servoName.toInt();
    uint8_t servoAngle = ival.toInt() + servoAdjustment[servoId];
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
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(1)\">Bow</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(2)\">Winken</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(7)\">Klatschen</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(3)\">Iron Man</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(4)\">Apache</button></td>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(5)\">Balance</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(6)\">Warm-Up</button></td>";
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
  
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].attach(servos[i].pin, i, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  }
  for(uint8_t i = PWM_SERVOS; i < SERVOS; i++) {
    isrServo[i & 3] = ESP32_ISR_Servos.setupServo(servos[i].pin, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  }

  // load idle position
  for(uint8_t i = 0; i < SERVOS; i++) {
    servoAdjustment[i] = preferences.getChar(servos[i].name);
    servoPosition[i] = idlePosition[i] + servoAdjustment[i];
    setAngle(i, servoPosition[i]);
  }
  servoDelay = preferences.getUInt(SERVO_DELAY);
  delay(1000);
}

void loop() {
  server.handleClient();

  if (servoProgram >= 1) {
    Serial.println("servoProgram = " + String(servoProgram));

    switch (servoProgram) {
      case 1:
        servoProgramRun(moveForward, moveForwardSteps);
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
      case 2:
        servoProgramRun(waving, wavingSteps);
        servoProgramIdle();
        delay(300);
        break;
      case 7:
        servoProgramRun(clapHands, clapHandsSteps);
        servoProgramIdle();
        delay(300);
        break;
    }
    servoProgramStack = 0;
  }
}
