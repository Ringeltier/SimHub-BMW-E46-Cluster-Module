// Includes for program
#include <SPI.h>
#include <mcp2515.h>
#include "constants.h"
#include "pin_definitions.h"
#include <CustomSoftwareSerial.h>

// C File Includes
#include "cluster_control_functions.h"

// Defines for program
#define DEBUG_VALUES true

// MCP2515 Instantiation
#define CS_PIN 9
#define IRQ_PIN 2  // Optional, only needed if you're using interrupts

MCP2515 CAN(CS_PIN);
CustomSoftwareSerial kbus(255, 7);

// Global variables that are changed in the main loop, and initializing certian states
unsigned long lastMsgTime = 0;
uint8_t fuel_val;
uint8_t abs_warning_state;
int speed_val;
uint8_t ignition_state;
uint8_t accessory_state;

byte gearIndex = 0;
uint16_t fuelConsumptionCounter = 0;
unsigned long lastMPGUpdate = 0;
float rpm_val;
int temp_val;
int mpg_val;
int engine_light_state;
int cruise_light_state;
int EML_state;
int gas_cap_light_state;
int heat_light_state;
int charge_light_state;
int park_brake_state;
int oil_light_state;
bool high_beam_state;
bool gearbox_state = 1;
bool fog_rear_state;
bool fog_front_state;
bool left_blinker_state;
bool right_blinker_state;
bool back_right_light_state;
bool back_left_light_state;
bool front_right_light_state;
bool front_left_light_state;
bool car_light_state;
bool light_state;
bool asc_state;
bool airbag_state;
int lightIndex = 0;
unsigned long lastLight = 0;

int fuel_consumption_val;
int rate_var;
int loopCounter = 0;
int lastGearTime = 0;


void setup() {

  // Sensor Initialization
  accessory_state = 0;
  ignition_state = 0;
  fuel_val = 50;
  park_brake_state = 1;
  abs_warning_state = 0;
  airbag_state = 0;
  asc_state = 1;
  temp_val = 275;
  rpm_val = 1000;

  // E46 Cluster Control Pin Setup
  pinMode(PARKING_BRAKE_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(ABS_WARNING_PIN, OUTPUT);
  pinMode(AIRBAG_PIN, OUTPUT);
  pinMode(SPEED_SIGNAL_PIN, OUTPUT);
  pinMode(IGNITION_ON_PIN, OUTPUT);
  pinMode(ACCESSORY_MODE_PIN, OUTPUT);

  // Set SPI Select pins low to start
  //digitalWrite(DIGI_POT_CS, HIGH);
  kbus.begin(9600, CSERIAL_8E1); 

  // Setup SPI for communication
  SPI.begin();

  // Serial Debug Communication
  Serial.begin(115200);  // Use fast baud for clearer debugging
  while (!Serial);       // Wait for Serial to be ready on boards like Leonardo

  Serial.println("Initializing CAN...");

  CAN.reset();  // Reset controller

  // Set bitrate to 500 kbps for 16 MHz crystal
  if (CAN.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    Serial.println("Bitrate set to 500kbps.");
  } else {
    Serial.println("Failed to set CAN bitrate.");
    while (1);  // Stop if bitrate setting fails
  }

  // Set to normal operating mode
  if (CAN.setNormalMode() == MCP2515::ERROR_OK) {
    Serial.println("CAN controller set to Normal Mode.");
  } else {
    Serial.println("Failed to set CAN controller to Normal Mode.");
    while (1);  // Stop if mode setting fails
  }

  Serial.println("CAN initialization successful.");
}

void loop() {
  // Setup the serial monitor to inject values into the program
  if(Serial.available()) {
    String data_type = Serial.readStringUntil(':');

    if(data_type == "speed") {
      String arg_str = Serial.readStringUntil('\n');
      speed_val = arg_str.toInt();
    }
    else if(data_type == "rpm") {
      String arg_str = Serial.readStringUntil('\n');
      rpm_val = arg_str.toInt();
    }
    else if(data_type == "temp") {
      String arg_str = Serial.readStringUntil('\n');
      temp_val = arg_str.toInt();
    }
    else if (data_type == "ignition") {
      String arg_str = Serial.readStringUntil('\n');
      ignition_state = arg_str.toInt();
    }
    else if (data_type == "accessory") {
      String arg_str = Serial.readStringUntil('\n');
      accessory_state = arg_str.toInt();
    }
    else if(data_type == "fuel") {
      String arg_str = Serial.readStringUntil('\n');
      fuel_val = arg_str.toInt();
    }
    else if (data_type == "ABS_Warning") {
      String arg_str = Serial.readStringUntil('\n');
      abs_warning_state = arg_str.toInt();
    }
    else if (data_type == "airbag") {
      String arg_str = Serial.readStringUntil('\n');
      airbag_state = arg_str.toInt();
    }
    else if(data_type == "eLight") {
      String arg_str = Serial.readStringUntil('\n');
      engine_light_state = arg_str.toInt();
    }
    else if(data_type == "emlLight") {
      String arg_str = Serial.readStringUntil('\n');
      EML_state = arg_str.toInt();
    }
    else if(data_type == "gasCapLight") {
      String arg_str = Serial.readStringUntil('\n');
      gas_cap_light_state = arg_str.toInt();
    }
    else if(data_type == "heatLight") {
      String arg_str = Serial.readStringUntil('\n');
      heat_light_state = arg_str.toInt();
    }
    else if(data_type == "oilLight") {
      String arg_str = Serial.readStringUntil('\n');
      oil_light_state = arg_str.toInt();
    }
    else if(data_type == "chgLight") {
      String arg_str = Serial.readStringUntil('\n');
      charge_light_state = arg_str.toInt();
    }
    else if(data_type == "hbrakeLight") {
      String arg_str = Serial.readStringUntil('\n');
      park_brake_state = arg_str.toInt();
    }
    else if(data_type == "highBeam") {
      String arg_str = Serial.readStringUntil('\n');
      high_beam_state = arg_str.toInt();
    }
    else if(data_type == "gearbox") {
      String arg_str = Serial.readStringUntil('\n');
      gearbox_state = arg_str.toInt();
    }
    else if(data_type == "fogRear") {
      String arg_str = Serial.readStringUntil('\n');
      fog_rear_state = arg_str.toInt();
    }
    else if(data_type == "fogFront") {
      String arg_str = Serial.readStringUntil('\n');
      fog_front_state = arg_str.toInt();
    }
    else if(data_type == "leftBlink") {
      String arg_str = Serial.readStringUntil('\n');
      left_blinker_state = arg_str.toInt();
    }
    else if(data_type == "rightBlink") {
      String arg_str = Serial.readStringUntil('\n');
      right_blinker_state = arg_str.toInt();
    }
    else if(data_type == "backRightLight") {
      String arg_str = Serial.readStringUntil('\n');
      back_right_light_state = arg_str.toInt();
    }
    else if(data_type == "backLeftLight") {
      String arg_str = Serial.readStringUntil('\n');
      back_left_light_state = arg_str.toInt();
    }
    else if(data_type == "frontRightLight") {
      String arg_str = Serial.readStringUntil('\n');
      front_right_light_state = arg_str.toInt();
    }
    else if(data_type == "frontLeftLight") {
      String arg_str = Serial.readStringUntil('\n');
      front_left_light_state = arg_str.toInt();
    }
    else if(data_type == "carLight") {
      String arg_str = Serial.readStringUntil('\n');
      car_light_state = arg_str.toInt();
    }
    else if(data_type == "light") {
      String arg_str = Serial.readStringUntil('\n');
      light_state = arg_str.toInt();
    }
    else if(data_type == "ascLight") {
      String arg_str = Serial.readStringUntil('\n');
      asc_state = arg_str.toInt();
    }
    else {
      // Flush the rest of the data
      Serial.print('\n');
    }

      // Debugger Print out
#if DEBUG_VALUES
    Serial.print("fuel: ");
    Serial.println(fuel_val);

    Serial.print("ABS_Warning: ");
    Serial.println(abs_warning_state);

    Serial.print("airbag: ");
    Serial.println(airbag_state);

    Serial.print("ignition: ");
    Serial.println(ignition_state);

    Serial.print("accessory: ");
    Serial.println(accessory_state);
    
    Serial.print("rpm: ");
    Serial.println(rpm_val);
    
    Serial.print("speed: ");
    Serial.println(speed_val);
    
    Serial.print("temp: ");
    Serial.println(temp_val);
    
    Serial.print("mpg: ");
    Serial.println(mpg_val);

    Serial.print("Temperature Val: ");
    Serial.println(temp_val);

    Serial.print("eLight: ");
    Serial.println(engine_light_state);

    Serial.print("emlLight: ");
    Serial.println(EML_state);

    Serial.print("gasCapLight: ");
    Serial.println(gas_cap_light_state);

    Serial.print("heatLight: ");
    Serial.println(heat_light_state);

    Serial.print("oilLight: ");
    Serial.println(oil_light_state);

    Serial.print("chgLight: ");
    Serial.println(charge_light_state);

    Serial.print("hbrakeLight: ");
    Serial.println(park_brake_state);

    Serial.print("highBeam: ");
    Serial.println(high_beam_state);

    Serial.print("gearbox: ");
    Serial.println(gearbox_state);

    Serial.print("fogRear: ");
    Serial.println(fog_rear_state);

    Serial.print("fogFront: ");
    Serial.println(fog_front_state);

    Serial.print("leftBlink: ");
    Serial.println(left_blinker_state);

    Serial.print("rightBlink: ");
    Serial.println(right_blinker_state);

    Serial.print("backRightLight: ");
    Serial.println(back_right_light_state);

    Serial.print("backLeftLight: ");
    Serial.println(back_left_light_state);

    Serial.print("frontRightLight: ");
    Serial.println(front_right_light_state);

    Serial.print("frontLeftLight: ");
    Serial.println(front_left_light_state);

    Serial.print("carLight: ");
    Serial.println(car_light_state);

    Serial.print("light: ");
    Serial.println(light_state);

    Serial.print("ascLight: ");
    Serial.println(asc_state);
    
    Serial.println();
    
#endif
  }
  // Configure all of the digital outputs
  digitalWrite(ABS_WARNING_PIN, abs_warning_state);
  digitalWrite(AIRBAG_PIN, airbag_state);
  digitalWrite(IGNITION_ON_PIN, ignition_state);
  digitalWrite(ACCESSORY_MODE_PIN, accessory_state);
  digitalWrite(LIGHT_PIN, light_state);
  digitalWrite(PARKING_BRAKE_PIN, park_brake_state);

  
  if (millis() - lastGearTime > 2000) {
    gearIndex += 1;
    if(gearIndex == 5) {
      gearIndex = 0;
    }
    lastGearTime = millis();
  }
  sendGear(CAN, gearIndex, gearbox_state);
  
  // setFuel(DIGI_POT_CS, fuel_val); NOT READY YET
  setRPM(CAN, rpm_val);
  delay(2);
  setSpeedometer(CAN, SPEED_SIGNAL_PIN, speed_val);
  delay(2);
  setTemp(CAN, temp_val);
  delay(2);
  sendASC(CAN, speed_val, asc_state);
  delay(2);
  // Simulate fuel usage based on speed and rpm
  unsigned long now = millis();
  if (now - lastMPGUpdate > 20) { // update 50x per second
    lastMPGUpdate = now;

    // Simulate really aggressive fuel use
    int usage = map(speed_val, 0, 240, 5, 180) + map(rpm_val, 600, 6000, 5, 220);

    // Optional boost: simulate full throttle load
    if (rpm_val > 5000 || speed_val > 180) {
      usage += 100;
    }

    fuelConsumptionCounter += usage;
  }

  set_status_lights1(
    CAN,
    engine_light_state,
    cruise_light_state,
    EML_state,
    gas_cap_light_state,
    heat_light_state,
    oil_light_state,
    charge_light_state,
    fuelConsumptionCounter
  );


  struct can_frame incoming;

  // Check for any incoming CAN messages
  if (CAN.readMessage(&incoming) == MCP2515::ERROR_OK) {
    
    // 🧠 Look for the 0x615 "request" frame from IKE
    if (incoming.can_id == 0x615) {
      // Optional: debug print
      // 📤 Send your response — handbrake OFF
      //sendClusterFeedback(CAN, 000016, 0x2A, temp_val, park_brake_state); // NO VISIBLE CHANGE YET
    }
  }

  if(loopCounter >=50) {
    // Packing up data to send to kbus function
    bool light_array1[] = {high_beam_state, fog_rear_state, fog_front_state, left_blinker_state, right_blinker_state};
    bool light_array2[] = {back_right_light_state, back_left_light_state, front_right_light_state, front_left_light_state, car_light_state};
    set_kbus_lights(kbus, light_array1, light_array2);
    // Reset counter
    loopCounter = 0;
  }
  loopCounter += 1;
  delay(1);
}
