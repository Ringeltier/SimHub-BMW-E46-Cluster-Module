// Updated cluster_control_functions.cpp with improved function naming
#include <Arduino.h>
#include <mcp2515.h>
#include "constants.h"
#include <CustomSoftwareSerial.h>

uint16_t fuel_consumption_total = 0;
byte iso_checksum(byte *data, byte len) {
  byte crc = 0;
  for (byte i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

void sendKbusMessage(CustomSoftwareSerial &refSer, byte *data) {
  int end_i = data[1] + 2;
  data[end_i - 1] = iso_checksum(data, end_i - 1);
  refSer.write(data, end_i + 1);
}

void sendCanMessage(MCP2515 &can_bus, uint16_t address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
  struct can_frame canMsg;
  canMsg.can_id = address;
  canMsg.can_dlc = 8;
  canMsg.data[0] = a;
  canMsg.data[1] = b;
  canMsg.data[2] = c;
  canMsg.data[3] = d;
  canMsg.data[4] = e;
  canMsg.data[5] = f;
  canMsg.data[6] = g;
  canMsg.data[7] = h;

  const int maxRetries = 5;
  byte result;
  int attempt = 0;
  do {
    result = can_bus.sendMessage(&canMsg);
    if (result == MCP2515::ERROR_OK) return;
    delay(10);
    attempt++;
  } while (attempt < maxRetries);
}

void updateSpeedNeedle(MCP2515 &can_bus, uint8_t speed_signal_pin, int speed_kmh) {
  if (speed_kmh < MIN_SPEED_KMH) {
    noTone(speed_signal_pin);
    return;
  }
  if (speed_kmh > MAX_SPEED_KMH) {
    speed_kmh = MAX_SPEED_KMH;
  }

  // Improved frequency mapping — tuned for E39 cluster tone input
  float tone_freq;
  if (speed_kmh <= 20) {
    tone_freq = 45 + speed_kmh * 3.6;
  } else if (speed_kmh <= 40) {
    tone_freq = 130 + (speed_kmh - 20) * 5.6;  // was 5.2
  } else {
    tone_freq = 238 + (speed_kmh - 40) * 7.5;  // was 6.8
  }

  tone(speed_signal_pin, static_cast<int>(tone_freq));
}

void updateRPMNeedle(MCP2515 &can_bus, int rpm_val) {
  if (rpm_val > MAX_RPM) rpm_val = MAX_RPM;
  if (rpm_val < MIN_RPM) rpm_val = MIN_RPM;

  int rpm_scaled = int(RPM_GAIN * rpm_val) + RPM_OFFSET;
  int rpm_lsb = rpm_scaled & 0x00FF;
  int rpm_msb = (rpm_scaled >> 8) & 0xFF;

  sendCanMessage(can_bus, 0x316, 0, 0, rpm_lsb, rpm_msb, 0, 0, 0, 0);
}

void updateTemperatureGauge(MCP2515 &can_bus, int temp_val) {
  if (temp_val < MIN_TEMP) temp_val = MIN_TEMP;
  if (temp_val > MAX_TEMP) temp_val = MAX_TEMP;
  uint8_t temp_byte;
  if (temp_val <= 90) {
    temp_byte = map(temp_val, 70, 90, 0xA0, 0xB4); // cold to center
  } else if (temp_val <= 95) {
    temp_byte = 0xB4; // stabilized middle
  } else {
    temp_byte = map(temp_val, 100, 130, 225, 255);
  }

  // Use realistic CAN message for 0x329
  sendCanMessage(can_bus, 0x329,
    0x4F,           // Byte0: Bit1 set = TCU present
    temp_byte,      // Byte1: Coolant temp
    0x82,           // Byte2: Ambient pressure (safe value)
    0x10,           // Byte3: Battery OK
    0x00,           // Byte4: Engine running
    0x20,           // Byte5: Throttle %
    0xFF,           // Byte6: Fuel type = gasoline
    0x00            // Byte7: filler
  );
}

void updateKbusLights(CustomSoftwareSerial &refSer, bool *light_array1, bool *light_array2) {
  byte LightByte1 = 0x00;
  byte LightByte2 = 0x00;

  for (int i = 0; i < 5; i++) {
    if (light_array1[i]) LightByte1 |= 1 << (i + 2);
    if (light_array2[i]) LightByte2 |= 1 << (i + 2);
  }

  if (light_array2[4]) LightByte2 |= 1 << 7;

  byte light_array[] = {0xD0, 0x08, 0xBF, 0x5B, LightByte1, 0x00, 0x00, LightByte2, 0x00, 0x58, 0x00};
  sendKbusMessage(refSer, light_array);

  byte msg[] = {0x80, 0x04, 0xC0, 0x21, 0x00, 0x00};
  sendKbusMessage(refSer, msg);
}

void updateWarningIndicators(MCP2515 &can_bus, int engine_light_state, int cruise_light_state, int EML_state,
                             int gas_cap_light_state, int heat_light_state, int oil_light_state,
                             int charge_light_state, float throttle, int temp_val, float rpm_val, float speed_val) {
  uint8_t status1_byte = 0x00;
  if (engine_light_state) status1_byte |= (1 << 1);
  if (cruise_light_state) status1_byte |= (1 << 3);
  if (EML_state) status1_byte |= (1 << 4);
  if (gas_cap_light_state) status1_byte |= (1 << 6);

  // Base fuel consumption from speed & RPM
  int fuel_consumption =
    map(throttle, 0, 100, 0, 220); // Throttle dominates

  // RPM and speed only lightly contribute
  fuel_consumption += map(rpm_val, 600, 6000, 1, 30);
  fuel_consumption += map(speed_val, 0, 240, 1, 25);

  // Extra fuel usage for aggressive driving
  if (rpm_val > 5000 || speed_val > 180) fuel_consumption += 50;

  fuel_consumption_total += fuel_consumption;
  
  uint8_t fuel_LSB = fuel_consumption_total & 0xFF;
  uint8_t fuel_MSB = (fuel_consumption_total >> 8) & 0xFF;

  if (temp_val >= (MAX_TEMP-10)) heat_light_state = 1;

  int status_byte2 = (heat_light_state << 3) | (oil_light_state << 1);

  sendCanMessage(can_bus, 0x545, status1_byte, fuel_LSB, fuel_MSB, status_byte2, 0x00, charge_light_state, 0x00, 0x00);
}

void updateASCLight(MCP2515 &can, float speedKph, bool asc_state) {
  if (asc_state) {
    uint16_t speed_val = (uint16_t)(speedKph * 256 / 8);
    sendCanMessage(can, 0x153, 0x00, speed_val & 0xFF, (speed_val >> 8) & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
}

void updateGearDisplay(MCP2515 &can_bus, char gearChar, bool transmissionStatus) {
  if (!transmissionStatus) return;

  uint8_t gear_code;

  switch (gearChar) {
    case 'P': gear_code = 0x08; break;
    case 'R': gear_code = 0x07; break;
    case 'N': gear_code = 0x06; break;
    case 'D': gear_code = 0x05; break;
    case '4': gear_code = 0x04; break;
    case '3': gear_code = 0x03; break;
    case '2': gear_code = 0x02; break;
    case '1': gear_code = 0x01; break;
    case '5': gear_code = 0x09; break;
    case 'O': gear_code = 0x0A; break; // display none without error
    default:  return; // invalid input
  }

  sendCanMessage(can_bus, 0x43F,
    0x81,        // Byte0
    gear_code,   // Byte1: Gear selector
    0xFF,        // Byte2: no mode
    0xFF,        // Byte3
    0x00,        // Byte4
    0x80,        // Byte5: no warning
    0xFF,        // Byte6
    0x00         // Byte7
  );
}