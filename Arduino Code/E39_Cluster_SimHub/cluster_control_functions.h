
#ifndef CLUSTER_CONTROL_FUNCTIONS_H
#define CLUSTER_CONTROL_FUNCTIONS_H

#include <mcp2515.h>
#include <CustomSoftwareSerial.h>

// Core CAN and K-Bus message functions
void sendCanMessage(MCP2515 &can_bus, uint16_t address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h);
void sendKbusMessage(CustomSoftwareSerial &refSer, byte *data);

// Display + Gauge Updates
void updateSpeedNeedle(MCP2515 &can_bus, uint8_t speed_signal_pin, int speed_kmh);
void updateRPMNeedle(MCP2515 &can_bus, int rpm_val);
void updateTemperatureGauge(MCP2515 &can_bus, int temp_val);

// Gear and Shifter Display
void updateGearDisplay(MCP2515 &can_bus, char gearChar, bool transmissionStatus);

// Status Indicators (Cluster lights over CAN)
void updateWarningIndicators(
  MCP2515 &can_bus,
  int engine_light_state,
  int cruise_light_state,
  int EML_state,
  int gas_cap_light_state,
  int heat_light_state,
  int oil_light_state,
  int charge_light_state,
  float throttle,
  int temp_val,
  float rpm_val,
  float speed_val
);

// ASC/ESP message
void updateASCLight(MCP2515 &can, float speedKph, bool asc_state);

// K-Bus Light Updates
void updateKbusLights(CustomSoftwareSerial &refSer, bool *light_array1, bool *light_array2);

#endif

