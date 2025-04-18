// Refactored Arduino code for SimHub integration with strict input validation
#include <SPI.h>
#include <mcp2515.h>
#include "constants.h"
#include "pin_definitions.h"
#include <CustomSoftwareSerial.h>
#include "cluster_control_functions.h"

MCP2515 CAN(CS_PIN);
CustomSoftwareSerial kbus(255, K_BUS_PIN);

// Core cluster data
float rpm = 0;
float throttle = 0;
int speed_kmh = 0;
int temp_c = 0;
uint8_t fuel_percent = 50;
char gear = 'P';

// Ignition & electrical state
bool ignition_on = false;
bool accessory_on = false;

// Warning & status lights
bool abs_warning = false;
bool airbag_warning = false;
bool parking_brake_engaged = true;
bool engine_light = false;
bool cruise_control = false;
bool eml_warning = false;
bool gas_cap_warning = false;
bool high_temp_warning = false;
bool oil_warning = false;
bool battery_warning = false;
bool gearbox_present = true;
bool asc_enabled = false;

// Light indicators
bool high_beam, fog_rear, fog_front;
bool blinker_left, blinker_right;
bool tail_right, tail_left;
bool head_right, head_left;
bool interior_light, light_switch;

String serialBuffer = "";
unsigned long last_kbus_update = 0;
unsigned long last_can_update = 0;

void setup() {
  pinMode(PARKING_BRAKE_PIN, OUTPUT);
  pinMode(ABS_WARNING_PIN, OUTPUT);
  pinMode(AIRBAG_PIN, OUTPUT);
  pinMode(SPEED_SIGNAL_PIN, OUTPUT);
  pinMode(IGNITION_ON_PIN, OUTPUT);
  pinMode(ACCESSORY_MODE_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  kbus.begin(9600, CSERIAL_8E1);
  SPI.begin();
  Serial.begin(115200);

  CAN.reset();
  CAN.setBitrate(CAN_500KBPS, MCP_8MHZ);
  CAN.setNormalMode();
}

bool isValidFloat(String str) {
  bool dotSeen = false;
  for (unsigned int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (i == 0 && (c == '-' || c == '+')) continue;
    if (c == '.') {
      if (dotSeen) return false;
      dotSeen = true;
    } else if (!isDigit(c)) {
      return false;
    }
  }
  return true;
}

bool isBool(String str) {
  return str == "0" || str == "1";
}

void parseSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuffer.trim();

      // Process all frames in buffer
      int start = 0;
      while ((start = serialBuffer.indexOf('<')) != -1) {
        int end = serialBuffer.indexOf('>', start);
        if (end == -1) break;

        String frame = serialBuffer.substring(start + 1, end);
        serialBuffer = serialBuffer.substring(end + 1); // trim processed

        int sep = frame.indexOf(':');
        if (sep > 0 && sep < frame.length() - 1) {
          String key = frame.substring(0, sep);
          String valStr = frame.substring(sep + 1);
          valStr.trim();

          if (key == "GEAR" && valStr.length() > 0) {
            gear = valStr[0];
          } else if (isBool(valStr)) {
            bool flag = valStr == "1";
            if (key == "IGN") ignition_on = flag;
            else if (key == "ACC") accessory_on = flag;
            else if (key == "ABS") abs_warning = flag;
            else if (key == "AIRBAG") airbag_warning = flag;
            else if (key == "PARK") parking_brake_engaged = flag;
            else if (key == "HIGH") high_beam = flag;
            else if (key == "FOGF") fog_rear = flag;
            else if (key == "FOGR") fog_front = flag;
            else if (key == "LEFT") blinker_left = flag;
            else if (key == "RIGHT") blinker_right = flag;
            else if (key == "BRR") tail_right = flag;
            else if (key == "BRL") tail_left = flag;
            else if (key == "FRR") head_right = flag;
            else if (key == "FRL") head_left = flag;
            else if (key == "CAR") interior_light = flag;
            else if (key == "LIGHT") light_switch = flag;
            else if (key == "ASC") asc_enabled = flag;
            else if (key == "ENG") engine_light = flag;
            else if (key == "EML") eml_warning = flag;
            else if (key == "GASCAP") gas_cap_warning = flag;
            else if (key == "HEAT") high_temp_warning = flag;
            else if (key == "OIL") oil_warning = flag;
            else if (key == "CHG") battery_warning = flag;
            else if (key == "GEARBOX") gearbox_present = flag;
          } else if (isValidFloat(valStr)) {
            float val = valStr.toFloat();
            if (key == "RPM") rpm = val;
            else if (key == "SPEED") speed_kmh = val;
            else if (key == "TEMP") temp_c = val;
            else if (key == "FUEL") fuel_percent = val;
            else if (key == "THROTTLE") throttle = val;
          } else {
            Serial.println("Invalid frame: <" + key + ":" + valStr + ">");
          }
          Serial.println("Parsed: " + key + "=" + valStr);
        }
      }

      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

void loop() {
  parseSerial();

  digitalWrite(ABS_WARNING_PIN, abs_warning);
  digitalWrite(AIRBAG_PIN, airbag_warning ? LOW : HIGH);
  digitalWrite(IGNITION_ON_PIN, ignition_on ? LOW : HIGH);
  digitalWrite(ACCESSORY_MODE_PIN, accessory_on ? LOW : HIGH);
  digitalWrite(LIGHT_PIN, light_switch ? LOW : HIGH);
  digitalWrite(PARKING_BRAKE_PIN, parking_brake_engaged ? LOW : HIGH);

  if (millis() - last_can_update > 5) {
    updateGearDisplay(CAN, gear, gearbox_present);
    updateRPMNeedle(CAN, rpm);
    updateSpeedNeedle(CAN, SPEED_SIGNAL_PIN, speed_kmh);
    updateTemperatureGauge(CAN, temp_c);
    updateASCLight(CAN, speed_kmh, asc_enabled ? 0 : 1);
    updateWarningIndicators(CAN, engine_light, cruise_control, eml_warning, gas_cap_warning,
                            high_temp_warning, oil_warning, battery_warning,
                            throttle, temp_c, rpm, speed_kmh);
    last_can_update = millis();
  }

  if (millis() - last_kbus_update > 50) {
    bool lights1[] = {high_beam, fog_rear, fog_front, blinker_left, blinker_right};
    bool lights2[] = {tail_right, tail_left, head_right, head_left, interior_light};
    updateKbusLights(kbus, lights1, lights2);
    last_kbus_update = millis();
  }
}