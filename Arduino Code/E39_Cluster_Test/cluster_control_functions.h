// Extern references to get these functions to the main program
extern void setSpeedometer(MCP2515 &can_bus, uint8_t speed_signal_pin, int speed_val);
extern void sendKeepAlive(MCP2515 &can_bus);
extern void setRPM(MCP2515 &can_bus, int rpm_val);
extern void setTemp(MCP2515 &can_bus, int temp_val);
extern void setFuel(uint8_t chip_select_pin, uint8_t fuel_percent);
extern void set_kbus_lights(CustomSoftwareSerial &refSer, bool *light_array1, bool *light_array2);
extern void set_status_lights1(MCP2515 &can_bus, int engine_light_state, int cruise_light_state, int EML_state, int gas_cap_light_state, int heat_light_state, int oil_light_state, int charge_light_state, int fuel_consumption);
extern void sendASC(MCP2515 &can, float speedKph, bool asc_state);
extern void sendClusterFeedback(MCP2515 &can, int odoKm = 000016, int fuelHex = 0x2A, int tempC = 20, int handbrakeOn = 0);
extern void setMPG(int mpg_val);
extern void sendGear(MCP2515 &can_bus, uint8_t gearIndex, bool transmissionStatus);