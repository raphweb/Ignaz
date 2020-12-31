#include <Servo.h>
#include <ESP32_ISR_Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

#define PWM_SERVOS 16
#define GPIO_SERVOS 4
#define SERVOS PWM_SERVOS + GPIO_SERVOS

#define BASEDELAYTIME 10

#define MAX_PULSE_LENGTH 2450.0
#define MIN_PULSE_LENGTH  550.0

const char* ssid = "WLAN";
const char* password = "****";
WebServer server(80);

// Motion Data Index
int servoProgram;
int servoProgramStack;

const uint8_t servoPins[] = {
// PWMs
// 0   1   2  3   4   5  6   7   8   9  10  11 12  13  14  15
  21, 19, 18, 5, 17, 16, 4, 12, 14, 27, 26, 25,33, 32, 22, 23,
// GPIOs 
  13, 3, 1, 0};
Servo servo[PWM_SERVOS];
uint8_t isrServo[GPIO_SERVOS];

uint8_t servoPosition[SERVOS];

const uint8_t zeroPosition[SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5   6   7   8    9  10   11  12   13  14   15  16  17  18  19 ms
  70, 90, 60, 120, 60, 145, 60, 90, 90, 120, 35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 0
};

const uint8_t idlePosition[SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5    6   7   8   9  10   11  12   13  14   15  16  17  18  19 ms
  70, 90, 60, 120, 60, 145, 135, 90, 90, 45, 35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 0
};

const uint8_t clapHandsSteps = 10;
uint8_t clapHands [clapHandsSteps][SERVOS + 1] PROGMEM = {
// 0   1   2    3   4    5    6   7    8   9   10   11  12   13  14   15  16  17  18  19 10ms
{ 70, 90, 60, 120, 60, 145, 135, 90,  90, 45,  35, 120, 60, 120, 90, 110, 90, 90, 90, 90,  5 },  // 1
{ 70, 90, 60, 120, 60,  55, 135, 90,  90, 45, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 45 },  // 2
{ 70, 90, 60, 120, 60,  55, 135, 44, 136, 45, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 15 },  // 3
{ 70, 90, 60, 120, 60,  55, 165, 44, 136, 15, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 },  // 4
{ 70, 90, 60, 120, 60,  55, 145, 44, 136, 35, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 },  // 5
{ 70, 90, 60, 120, 60,  55, 165, 44, 136, 15, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 },  // 6
{ 70, 90, 60, 120, 60,  55, 145, 44, 136, 35, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 },  // 7
{ 70, 90, 60, 120, 60,  55, 165, 44, 136, 15, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 },  // 8
{ 70, 90, 60, 120, 60,  55, 145, 44, 136, 35, 118, 120, 60, 120, 90, 110, 90, 90, 90, 90, 25 },  // 9
{ 70, 90, 60, 120, 60, 145, 135, 90,  90, 45,  35, 120, 60, 120, 90, 110, 90, 90, 90, 90, 50 }   // 10
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
    servoPosition[i] = zeroPosition[i] + (int8_t)EEPROM.read(i);
  }
  for(uint8_t iServo = 0; iServo < SERVOS; iServo++) {
    setAngle(iServo, servoPosition[iServo]);
    delay(BASEDELAYTIME);
  }
}

void servoProgramIdle() {
  for(uint8_t i = 0; i < SERVOS; i++) {
    servoPosition[i] = idlePosition[i] + (int8_t)EEPROM.read(i);
  }
  for(uint8_t iServo = 0; iServo < SERVOS; iServo++)
  {
    setAngle(iServo, servoPosition[iServo]);
    delay(BASEDELAYTIME);
  }
}

void servoProgramRun(uint8_t iMatrix[][SERVOS + 1], uint8_t iSteps) {
  int tmpa, tmpb, tmpc;
  for(uint8_t mainLoopIndex = 0; mainLoopIndex < iSteps; mainLoopIndex++) {
    int interTotalTime = iMatrix[mainLoopIndex][SERVOS] * BASEDELAYTIME + (int8_t)EEPROM.read(SERVOS);
    int interDelayCounter = interTotalTime / BASEDELAYTIME;
    for(uint8_t interStepLoop = 0; interStepLoop < interDelayCounter; interStepLoop++) {
      for(uint8_t servoIndex = 0; servoIndex < SERVOS; servoIndex++) {
        tmpa = servoPosition[servoIndex];
        tmpb = iMatrix[mainLoopIndex][servoIndex] + (int8_t)EEPROM.read(servoIndex);

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
      servoPosition[i] = iMatrix[mainLoopIndex][i] + (int8_t)EEPROM.read(i);
    }
  }
}

void writeKeyValue(uint8_t key, int8_t value) {
  EEPROM.write(key, value);
  EEPROM.commit();
  Serial.println("write (" + String(key) + ", " + String(value) + ")");
}

int8_t readKeyValue(uint8_t key) {
  const int8_t value = EEPROM.read(key);
  Serial.println("read (" + String(key) + ", " + String(value) + ")");
  return value;
}

void handleSave() {
  String key = server.arg("key");
  String value = server.arg("value");

  uint8_t keyInt = key.toInt();
  int8_t valueInt = value.toInt();

  // Software PWM PIN detach
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].detach();
  }
  delay(50);
  if (keyInt == 100) {
    for(uint8_t i = 0; i < SERVOS + 1; i++) {
      writeKeyValue(i, 0);
    }
  } else if (valueInt >= -125 && valueInt <= 125) {
    writeKeyValue(keyInt, valueInt);
  }

  // Software PWM PIN attach
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].attach(servoPins[i]);
  }
  delay(10);
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
    uint8_t servoID = servoName.toInt();
    String ival = server.arg("value");
    uint8_t servoAngle = ival.toInt() + (int8_t)EEPROM.read(servoID);
    setAngle(servoID, servoAngle);
  }

  server.send(200, "text/html", "(pm, pms)=(" + pm + "," + pms + ")");
}

void handleGetEEPROM() {
  String content = "";
  for(uint8_t i = 0; i < SERVOS - 1; i++) {
    content += String(readKeyValue(i)) + ", ";
  }
  content += String(readKeyValue(SERVOS - 1));
  server.send(200, "text/html", content);
}

void handleZero() {
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
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #6e6e6e;\" type=\"button\" onclick=\"controlServo(19, " + String(zeroPosition[19]) + ")\">Servo 19</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #6e6e6e;\" type=\"button\" onclick=\"controlServo(18, " + String(zeroPosition[18]) + ")\">Servo 18</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #6e6e6e;\" type=\"button\" onclick=\"controlServo(17, " + String(zeroPosition[17]) + ")\">Servo 17</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #6e6e6e;\" type=\"button\" onclick=\"controlServo(16, " + String(zeroPosition[16]) + ")\">Servo 16</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #f5da81;\" type=\"button\" onclick=\"controlServo(8, " + String(zeroPosition[8]) + ")\">Servo 8</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #f5da81;\" type=\"button\" onclick=\"controlServo(7, " + String(zeroPosition[7]) + ")\">Servo 7</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #bdbdbd;\" type=\"button\" onclick=\"controlServo(9, " + String(zeroPosition[9]) + ")\">Servo 9</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #bdbdbd;\" type=\"button\" onclick=\"controlServo(6, " + String(zeroPosition[6]) + ")\">Servo 6</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(10, " + String(zeroPosition[10]) + ")\">Servo 10</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(5,  " + String(zeroPosition[5])  + ")\">Servo  5</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(11, " + String(zeroPosition[11]) + ")\">Servo 11</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(4,  " + String(zeroPosition[4])  + ")\">Servo  4</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #bdbdbd;\" type=\"button\" onclick=\"controlServo(12, " + String(zeroPosition[12]) + ")\">Servo 12</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #bdbdbd;\" type=\"button\" onclick=\"controlServo(3,  " + String(zeroPosition[3])  + ")\">Servo  3</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #f5da81;\" type=\"button\" onclick=\"controlServo(13, " + String(zeroPosition[13]) + ")\">Servo 13</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #f5da81;\" type=\"button\" onclick=\"controlServo(2,  " + String(zeroPosition[2])  + ")\">Servo  2</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #bdbdbd;\" type=\"button\" onclick=\"controlServo(14, " + String(zeroPosition[14]) + ")\">Servo 14</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #bdbdbd;\" type=\"button\" onclick=\"controlServo(1,  " + String(zeroPosition[1])  + ")\">Servo  1</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(15, " + String(zeroPosition[15]) + ")\">Servo 15</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlServo(0,  " + String(zeroPosition[0])  + ")\">Servo  0</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(100)\">ALL</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

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
  String servoValStr[SERVOS + 1];
  for(uint8_t i = 0; i < SERVOS + 1; i++) {
    servoValStr[i] = String(readKeyValue(i));
  }

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
  content += "<tr>";
  content += "<td>Servo 19<br/><input class=\"pm_text\" type=\"text\" id=\"servo_19\" value=\"" + servoValStr[19] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(19,'servo_19')\">SET</button></td>";
  content += "<td>Servo 18<br/><input class=\"pm_text\" type=\"text\" id=\"servo_18\" value=\"" + servoValStr[18] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(18,'servo_18')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 17<br/><input class=\"pm_text\" type=\"text\" id=\"servo_17\" value=\"" + servoValStr[17] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(17,'servo_17')\">SET</button></td>";
  content += "<td>Servo 16<br/><input class=\"pm_text\" type=\"text\" id=\"servo_16\" value=\"" + servoValStr[16] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(16,'servo_16')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 8<br/><input class=\"pm_text\" type=\"text\" id=\"servo_8\" value=\"" + servoValStr[8] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(8,'servo_8')\">SET</button></td>";
  content += "<td>Servo 7<br/><input class=\"pm_text\" type=\"text\" id=\"servo_7\" value=\"" + servoValStr[7] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(7,'servo_7')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 9<br/><input class=\"pm_text\" type=\"text\" id=\"servo_9\" value=\"" + servoValStr[9] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(9,'servo_9')\">SET</button></td>";
  content += "<td>Servo 6<br/><input class=\"pm_text\" type=\"text\" id=\"servo_6\" value=\"" + servoValStr[6] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(6,'servo_6')\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 10<br/><input class=\"pm_text\" type=\"text\" id=\"servo_10\" value=\"" + servoValStr[10] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(10,'servo_10')\">SET</button></td>";
  content += "<td>Servo  5<br/><input class=\"pm_text\" type=\"text\" id=\"servo_5\"  value=\"" + servoValStr[5]  + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(5, 'servo_5' )\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 11<br/><input class=\"pm_text\" type=\"text\" id=\"servo_11\" value=\"" + servoValStr[11] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(11,'servo_11')\">SET</button></td>";
  content += "<td>Servo  4<br/><input class=\"pm_text\" type=\"text\" id=\"servo_4\"  value=\"" + servoValStr[4]  + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(4, 'servo_4' )\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 12<br/><input class=\"pm_text\" type=\"text\" id=\"servo_12\" value=\"" + servoValStr[12] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(12,'servo_12')\">SET</button></td>";
  content += "<td>Servo  3<br/><input class=\"pm_text\" type=\"text\" id=\"servo_3\"  value=\"" + servoValStr[3]  + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(3, 'servo_3' )\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 13<br/><input class=\"pm_text\" type=\"text\" id=\"servo_13\" value=\"" + servoValStr[13] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(13,'servo_13')\">SET</button></td>";
  content += "<td>Servo  2<br/><input class=\"pm_text\" type=\"text\" id=\"servo_2\"  value=\"" + servoValStr[2]  + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(2, 'servo_2' )\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 14<br/><input class=\"pm_text\" type=\"text\" id=\"servo_14\" value=\"" + servoValStr[14] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(14,'servo_14')\">SET</button></td>";
  content += "<td>Servo  1<br/><input class=\"pm_text\" type=\"text\" id=\"servo_1\"  value=\"" + servoValStr[1]  + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(1, 'servo_1' )\">SET</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td>Servo 15<br/><input class=\"pm_text\" type=\"text\" id=\"servo_15\" value=\"" + servoValStr[15] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(15,'servo_15')\">SET</button></td>";
  content += "<td>Servo  0<br/><input class=\"pm_text\" type=\"text\" id=\"servo_0\"  value=\"" + servoValStr[0]  + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(0, 'servo_0' )\">SET</button></td>";
  content += "</tr>";
  content += "</table>";

  content += "<br>";

  content += "<table align=center>";
  content += "<tr>";
  content += "<td>Delay Time<br/><input class=\"pm_text\" type=\"text\" id=\"servo_20\" value=\"" + servoValStr[20] + "\"><button class=\"pm_btn\" type=\"button\" onclick=\"saveServo(20,'servo_20')\">SET</button></td>";
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
  content += "    document.getElementById(\"servo_20\").value = \"0\";";
  content += "    document.getElementById(\"servo_19\").value = \"0\";";
  content += "    document.getElementById(\"servo_18\").value = \"0\";";
  content += "    document.getElementById(\"servo_17\").value = \"0\";";
  content += "    document.getElementById(\"servo_16\").value = \"0\";";
  content += "    document.getElementById(\"servo_15\").value = \"0\";";
  content += "    document.getElementById(\"servo_14\").value = \"0\";";
  content += "    document.getElementById(\"servo_13\").value = \"0\";";
  content += "    document.getElementById(\"servo_12\").value = \"0\";";
  content += "    document.getElementById(\"servo_11\").value = \"0\";";
  content += "    document.getElementById(\"servo_10\").value = \"0\";";
  content += "    document.getElementById(\"servo_9\").value = \"0\";";
  content += "    document.getElementById(\"servo_8\").value = \"0\";";
  content += "    document.getElementById(\"servo_7\").value = \"0\";";
  content += "    document.getElementById(\"servo_6\").value = \"0\";";
  content += "    document.getElementById(\"servo_5\").value = \"0\";";
  content += "    document.getElementById(\"servo_4\").value = \"0\";";
  content += "    document.getElementById(\"servo_3\").value = \"0\";";
  content += "    document.getElementById(\"servo_2\").value = \"0\";";
  content += "    document.getElementById(\"servo_1\").value = \"0\";";
  content += "    document.getElementById(\"servo_0\").value = \"0\";";
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
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(1)\">Forward</button></td>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(4)\">TurnRight</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td><button class=\"pm_btn\" type=\"button\" onclick=\"controlPm(5)\">MoveLeft</button></td>";
  content += "<td><button class=\"pm_btn\" style=\"background: #ed3db5;\" type=\"button\" onclick=\"controlPm(99)\">STANDBY</button></td>";
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
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(2)\">Waving</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button class=\"pms_btn\" style=\"background: #ffbf00;\" type=\"button\" onclick=\"controlPms(7)\">Clap Hands</button></td>";
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
  EEPROM.begin(SERVOS + 1);
  EEPROM.commit();
  
  Serial.begin(115200);
  while(!Serial) {};
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode( WIFI_STA );
  WiFi.begin(ssid, password);
  WiFi.setHostname("ignaz");
  WiFi.setAutoReconnect(true);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Wifi connection failed!");
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
  server.on("/eeprom", HTTP_GET, handleGetEEPROM);

  server.begin();
  Serial.println("HTTP server started");
  
  for(uint8_t i = 0; i < PWM_SERVOS; i++) {
    servo[i].attach(servoPins[i], MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  }
  for(uint8_t i = PWM_SERVOS; i < PWM_SERVOS + GPIO_SERVOS; i++) {
    isrServo[i & 3] = ESP32_ISR_Servos.setupServo(servoPins[i], MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  }

  // load idle position
  for(uint8_t i = 0; i < SERVOS; i++) {
    servoPosition[i] = idlePosition[i] + (int8_t)EEPROM.read(i);
    setAngle(i, servoPosition[i]);
  }
  delay(1000);
}

void loop() {
  server.handleClient();

  if (servoProgram >= 1) {
    Serial.print("servoProgram = ");
    Serial.println(servoProgram);

    switch (servoProgram) {
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
    Serial.print("servoProgramStack = ");
    Serial.println(servoProgramStack);

    switch (servoProgramStack) {
      case 7:
        servoProgramRun(clapHands, clapHandsSteps);
        servoProgramIdle();
        delay(300);
        break;
    }
    servoProgramStack = 0;
  }
}
