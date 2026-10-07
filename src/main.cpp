/**
 * @file main.cpp
 * @brief Main control program for KestrelSAT attitude determination and control system
 * 
 * @details This is firmware for the USAFA Astro 331 KestrelSAT, implementing:
 * - Lab 6 Attitude Determination capabilities using IMU (gyroscope, magnetometer) and sun sensors
 * - Lab 7 Attitude Control via reaction wheel motor with encoder feedback
 * - Ground station command interface via XBee radio and USB Serial
 * - Battery telemetry monitoring via MAX17048 fuel gauge
 * - Data logging to SD card for Lab 6 and Lab 7 experiments
 * - Lab 8 fan thrusters: manual control, thrust sweep, open-loop slew, RW momentum dump
 * 
 * @author Lt Col Wyatt Harris
 * @date Spring 2026
 * @version 2.0
 * 
 * @section hardware Hardware Requirements
 * - ESP32 microcontroller (SparkFun Thing Plus ESP32-WROOM)
 * - ICM-20948 9-DoF IMU (via I2C at 0x36)
 * - MAX17048 Fuel Gauge (via I2C at 0x36)
 * - Pololu TB9051FTG Motor Driver
 * - DC motor with quadrature encoder
 * - Sun sensor (4-channel analog inputs)
 * - XBee radio module (UART at 9600 baud)
 * - SD card module (SPI)
 * - WS2812 RGB LED and digital status LED
 * 
 * @section dependencies External Libraries
 * - greiman/SdFat@^2.3.1 - SD card file system
 * - sparkfun/SparkFun MAX1704x Fuel Gauge@^1.0.3 - Battery monitoring
 * - sparkfun/SparkFun 9DoF IMU (ICM 20948)@^1.3.2 - IMU sensor interface
 * - meelonusk/TB9051FTGMotorCarrier@^1.0.2 - Motor driver control
 * - madhephaestus/ESP32Encoder@^0.12.0 - Quadrature encoder reading
 * 
 * @section commands Ground Station Commands
 * | Cmd | Function |
 * |-----|----------|
 * | 0   | Stop reaction wheel and thrusters |
 * | 1   | Display options menu |
 * | 2   | Query XBee RSSI |
 * | 3   | (disabled in Lab 8: status LED pin GPIO13 drives the -Z fan) |
 * | 4   | Get battery telemetry (V, SOC, charge rate) |
 * | 5   | Manually set reaction wheel throttle (-100 to 100%) |
 * | 6   | Run Lab 6 test (Attitude Determination) |
 * | 7   | Run Lab 7 Test A (tabletop static, RW torque measurement) |
 * | 8   | Run Lab 7 Test B (dynamic test with prescribed wheel speed) |
 * | 9   | Stream reaction wheel speed |
 * | 10  | Lab 8: Manually set thrusters (-100 to 100%, + = +Z fan, - = -Z fan) |
 * | 11  | Lab 8: Thrust sweep (bench, each fan stepped 0-100%) |
 * | 12  | Lab 8: Open-loop thruster slew (string-hung) |
 * | 13  | Lab 8: RW momentum dump with thruster rate damping (string-hung) |
 * | 98  | SD Card: List files (USB Serial only) |
 * | 99  | SD Card: Print file menu (USB Serial only) |
 * 
 * @section testing Test Modes
 * - **Lab 6**: Collects IMU (gyro Z, mag X/Y) and sun sensor data at 50ms intervals
 * - **Lab 7A**: 15-second tabletop test with 100% motor step input from 3-10 seconds
 * - **Lab 7B**: 50-second dynamic test with prescribed motor speed profile (hold, ramp up/down cycles)
 * - **Lab 8 Sweep**: each fan held 5 s at 25/50/75/100% duty with 3 s off between steps (read thrust off a scale)
 * - **Lab 8 Slew**: 2 s idle, +Z burn, equal -Z burn to stop, then coast (open loop)
 * - **Lab 8 Dump**: RW spun to full speed, release on blue LED, RW ramped to zero while thrusters damp body rate
 * 
 * @section datalog Data Logging Format
 * All test data is logged to SD card in CSV format with:
 * - MCU timestamp (ms)
 * - Gyroscope Z-axis (deg/s)
 * - Magnetometer X, Y axes (µT)
 * - Sun direction (degrees)
 * - Sun sensor raw values (4 channels)
 * - Reaction wheel commanded and measured speed (RPM)
 * 
 * @section communication Communication
 * - **USB Serial**: 115200 baud for USB debugging/control
 * - **XBee UART**: 9600 baud for wireless ground station link
 * - **I2C**: Sensor communication at standard rate
 * - **SPI**: SD card access
 * 
 * @note Heartbeat signal sent every 2 seconds to indicate program status
 * @note Ground station commands checked every 10ms
 * @note RGB LED provides visual status: Red (startup), Green (idle), Orange (Lab 6), Cyan (Lab 7A), Magenta (Lab 7B),
 *       Yellow (Lab 8 sweep), White (Lab 8 slew / dump hold), Blue (Lab 8 dump: release KestrelSAT)
 * 
 * @warning All test loops use blocking serial reads; may delay program execution if waiting for input
 * @warning Reaction wheel tests should be conducted with appropriate safety precautions
 * 
 * @see definitions.h for hardware pin assignments and constants
 * @see sd_functions.h for SD card utility functions
 */

/*---------------------------------------------------------------------------------------------*/
// Library includes:
/*---------------------------------------------------------------------------------------------*/
#include <Arduino.h>                                      // Main Arduino library
#include <Wire.h>                                         // libray for I2C communication
#include <SPI.h>                                          // SPI communication library
#include <SdFat.h>                                        // SD Card library
#include "definitions.h"                                  // Project definitions (this directory)
#include "sd_functions.h"                                 //SD helper functions (this directory)
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // MAX17048 fuel gauge
#include <ICM_20948.h>                                    // Sparkfun IMU library
#include <TB9051FTGMotorCarrier.h>                        // Pololu Motor Carrier Library
#include <ESP32Encoder.h>                                 // Motor encoder library to measure wheel speed
#include "thrusters.h"                                    // Lab 8 fan thruster driver (this directory)

/*---------------------------------------------------------------------------------------------*/
// Globals:
/*---------------------------------------------------------------------------------------------*/
// Objects:
FsFile dataFile;   // data file object
HardwareSerial Xbee(2); // Serial object for communication with XBee 
SFE_MAX1704X lipo; // SparkFun Thing Plus ESP32-WROOM onboard fuel gauge (I2C addr 0x36)
ICM_20948_I2C imu_sensor; // IMU object
// Motor Variables/Object
constexpr uint8_t pwm1Pin{MOTOR_PWM_1_PIN}; // PWM1
constexpr uint8_t pwm2Pin{MOTOR_PWM_2_PIN}; // PWM2
TB9051FTGMotorCarrier driver{ pwm1Pin, pwm2Pin };// Instantiate TB9051FTGMotorCarrier
ESP32Encoder enc;

// Variables:
uint32_t timeLastCheckForCommand; // time of next Xbee check
uint32_t interval_CheckForCommand = 10; // time interval between Xbee/Serial checks (ms)
uint32_t timeLastHeartBeat; // time of last heartbeat (ms)
uint32_t interval_heartBeat = 3000; // interval between heartbeat (ms)

float sun_plusX, sun_minusX, sun_plusY, sun_minusY;
float sun_X = 0.0, sun_Y = 0.0, sun_direction = 0.0;
float S_mag;
int16_t n_sun_sensor_reads = 5; // number of readings to average for sun sensor test point

uint32_t timeNext_testPoint; // time of next test point (ms)
uint32_t interval_testPoint = 50; // time interval between test points (ms)

float gyro_Z = 0.0;
float mag_X = 0.0;
float mag_Y = 0.0;

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see defintiions after loop()):
/*---------------------------------------------------------------------------------------------*/
int get_command_from_ground_station();
void process_main_menu();
void get_sat_rssi();
void send_battery_telemetry();
void manual_set_RW_speed();
void lab6_run_test();
void lab7_run_test_A();
void lab7_run_test_B();
float set_speed_test_B(uint32_t t0);
void stream_RWspeed();
void manual_set_thrusters();
void lab8_thrust_sweep();
void lab8_thruster_slew();
void lab8_momentum_dump();

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200); // Begin Serial communication with computer
  while (!Serial) {delay(10);} // Wait for user to open Serial monitor before proceeding
  Serial.println("[INFO] Hello World!");

  Wire.begin(); // Initialize I2C communication

  // Initialize built-in RGB LED (WS2812) and STAT LED
  // #define RGB_BUILTIN  2
  pinMode(RGB_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 255, 0, 0); // Default to red (R=255, G=0, B=0)

  //----------------------------------------------
  // Initialize Serial link with XBee
  //----------------------------------------------
  Xbee.begin(9600,SERIAL_8N1, XBEE_RX, XBEE_TX);  // Begin MCU <> XBee communication
  Xbee.setTimeout(20);
  Xbee.println("[INFO] KestrelSAT online \npress 1 for options");
  // ----------------------------------------------

  //----------------------------------------------
  // Initialize SD Card
  //----------------------------------------------
  sd_init(SD_CS_PIN);
  //----------------------------------------------

  //----------------------------------------------
  // Initialize MAX17048 fuel gauge
  //----------------------------------------------
  if (!lipo.begin(Wire)) // Uses I2C address 0x36) 
  {
    Serial.println("[WARN] MAX17048 not detected on I2C (0x36). Battery telemetry (cmd 4) will be unavailable.");
  } else {
    lipo.quickStart();    // Improves initial SOC accuracy after boot. Returns 0 on success.
    Serial.println("[INFO] MAX17048 online.");
  }
  //----------------------------------------------

  //----------------------------------------------
  // Initialize ICM20948
  //----------------------------------------------
  if (imu_sensor.begin(Wire, 1) != ICM_20948_Stat_Ok) {
    Serial.println("[CAUTION] IMU not found.");
    while (1);
  } else{
    // // 1. Set to Continuous Mode for consistent sampling
    // imu_sensor.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    // // 2. Configure Sample Rate Divider (formula: 1125 / (1 + divider) Hz)
    // ICM_20948_smplrt_t mySmplrt;
    // mySmplrt.a = 1; // Accel divider 1: 1125 / (1+1) = ~562.5 Hz
    // mySmplrt.g = 1; // Gyro divider 1: 1125 / (1+1) = ~562.5 Hz
    // imu_sensor.setSampleRate(ICM_20948_Internal_Acc, mySmplrt);
    // imu_sensor.setSampleRate(ICM_20948_Internal_Gyr, mySmplrt);    
    Serial.println("[INFO] IMU Initialized.");
  }
  //----------------------------------------------

  //----------------------------------------------
  // Initialize Sun Sensor
  //----------------------------------------------
  analogReadResolution(12);

  //----------------------------------------------
  // Initialize Reaction Wheel
  //----------------------------------------------
  driver.enable(); // TB9051FTG Motor Driver
  driver.setOutput(0);
  enc.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B); // Motor Encoder
  enc.clearCount();
  //----------------------------------------------

  //----------------------------------------------
  // Initialize Thrusters
  //----------------------------------------------
  thrusters_init();
  //----------------------------------------------

  Serial.println("[INFO] SETUP COMPLETE.");
  Xbee.println("[INFO] SETUP COMPLETE.SEND '1' FOR OPTIONS.");
  neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //----------------------------------------------
  // Check for data from ground station
  // ----------------------------------------------
  if (timeLastCheckForCommand + interval_CheckForCommand < millis()) { // periodic Xbee send
    timeLastCheckForCommand = millis();
    process_main_menu();
  }

  if (timeLastHeartBeat + interval_heartBeat < millis()) { // periodic heartbeat to indicate program is alive
    timeLastHeartBeat = millis();
    Serial.println("[INFO] Send '1' for Options");
    Xbee.println("*");
    neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCITON DEFINITIONS:
/////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------------------------*/
// Get Command from Ground Station:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Gets command received from the ground station (USB or XBee).
 *
 * Reads a command string from USB or XBee and parses the command into 
 * an integer.
 *
 * @note Command strings are trimmed of leading/trailing whitespace before
 *       parsing. Commands are expected to be integers at the start of the
 *       string. Unrecognized commands are ignored.
 * 
 * @returns int of command received
 *
 * @warning This function uses blocking serial reads (readStringUntil())
 *          and may delay program execution if no command is received.
 *
 */
int get_command_from_ground_station()
{
  int received_int = 0; // default to no command
  if (Xbee.available())
  {
    String received_string = Xbee.readStringUntil('\n');
    received_string.trim();
    if (received_string.length() == 0) return -1;
    Serial.print("Received from XBee: ");
    Serial.println(received_string);
    received_int = received_string.toInt();
  }else if (Serial.available())
  {
    String received_string = Serial.readStringUntil('\n');
    received_string.trim();
    if (received_string.length() == 0) return -1;
    Serial.print("Received from Serial: ");
    Serial.println(received_string);
    received_int = received_string.toInt();
  }
  return received_int;
}

/*---------------------------------------------------------------------------------------------*/
// Process Main Menu:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Process and execute commands received from the ground station.
 *
 * Reads a command integer from the ground station (USB Serial or XBee),
 * parses it, and executes the corresponding action. Supported commands:
 *
 * - 0: Stop reaction wheel motor
 * - 1: Print options menu
 * - 2: Get satellite RSSI
 * - 3: Disabled (GPIO13 status LED pin now drives the -Z fan)
 * - 4: Get battery telemetry (voltage, SOC, charge rate)
 * - 5: Manually set reaction wheel speed (throttle %)
 * - 6: Run Lab 6 - Attitude Determination test
 * - 7: Run Lab 7 - Test A (tabletop static test)
 * - 8: Run Lab 7 - Test B
 * - 98: SD Card - List files (USB Serial only)
 * - 99: SD Card - Print file menu (USB Serial only)
 *
 * @note Command strings are trimmed of leading/trailing whitespace before
 *       parsing. Commands are expected to be integers. Unrecognized commands
 *       are ignored with a caution message.
 *
 * @return void
 *
 */
void process_main_menu() {
  if (!Xbee.available() && !Serial.available()) return;

  int received_int = get_command_from_ground_station();

  switch (received_int) {
    case 0:
      driver.setOutput(0);
      thrusters_off();
      Xbee.println("Motor and Thrusters Stopped.");
      Serial.println("Motor and Thrusters Stopped.");
      break;
    
    case 1:
      Xbee.print("0 - Stop reation wheel and thrusters\n");
      Xbee.print("1 - Print Options Menu\n");
      Xbee.print("2 - Get RSSI\n");
      Xbee.print("3 - (disabled: LED pin used by -Z fan)\n");
      Xbee.print("4 - Get Battery State (V, SOC, dSOC/dt) \n");
      Xbee.print("5 - Set Motor Throttle Percent (-100...100)\n");
      Xbee.print("6 - Lab 6: Run Test\n");
      Xbee.print("7 - Lab 7: Run Test A\n");
      Xbee.print("8 - Lab 7: Run Test B\n");
      Xbee.print("9 - Stream RW speed\n");
      Xbee.print("10 - Lab 8: Set Thrusters Percent (-100...100)\n");
      Xbee.print("11 - Lab 8: Thrust Sweep (bench)\n");
      Xbee.print("12 - Lab 8: Open-Loop Thruster Slew\n");
      Xbee.print("13 - Lab 8: RW Momentum Dump\n");
      Xbee.print("98 - SD Card: List Files (USB SERIAL ONLY)\n");
      Xbee.print("99 - SD Card: Print File Menu (USB SERIAL ONLY)\n");
      
      Serial.print("0 - Stop reation wheel and thrusters\n");
      Serial.print("1 - Print Options Menu\n");
      Serial.print("2 - Get RSSI\n");
      Serial.print("3 - (disabled: LED pin used by -Z fan)\n");
      Serial.print("4 - Get Battery State (V, SOC, dSOC/dt) \n");
      Serial.print("5 - Set Motor Throttle Percent (-100...100)\n");
      Serial.print("6 - Lab 6: Run Test\n");
      Serial.print("7 - Lab 7: Run Test A\n");
      Serial.print("8 - Lab 7: Run Test B\n");
      Serial.print("9 - Stream RW speed\n");
      Serial.print("10 - Lab 8: Set Thrusters Percent (-100...100)\n");
      Serial.print("11 - Lab 8: Thrust Sweep (bench)\n");
      Serial.print("12 - Lab 8: Open-Loop Thruster Slew\n");
      Serial.print("13 - Lab 8: RW Momentum Dump\n");
      Serial.print("98 - SD Card: List Files (USB SERIAL ONLY)\n");
      Serial.print("99 - SD Card: Print File Menu (USB SERIAL ONLY)\n");
      break;

    case 2:
      get_sat_rssi();
      break;

    case 3:
      // Lab 8: GPIO13 (LED_BUILTIN) now drives the -Z fan, so the LED toggle is disabled.
      Serial.println("[CAUTION] Command 3 disabled: status LED pin (GPIO13) drives the -Z fan.");
      Xbee.println("[CAUTION] Command 3 disabled: status LED pin (GPIO13) drives the -Z fan.");
      break;

    case 4:
      send_battery_telemetry();
      break;

    case 5:
      manual_set_RW_speed();
      break;

    case 6:
      lab6_run_test();
      break;

    case 7:
      lab7_run_test_A();
      break;

    case 8:
      lab7_run_test_B();
      break;

    case 9:
      stream_RWspeed();
      break;

    case 10:
      manual_set_thrusters();
      break;

    case 11:
      lab8_thrust_sweep();
      break;

    case 12:
      lab8_thruster_slew();
      break;

    case 13:
      lab8_momentum_dump();
      break;

    case 98:
      sd_listFiles("/", 0);
      break;

    case 99:
      sd_printFileMenu();
      break;

    default:
      Xbee.println("[CAUTION] Invalid input from ground station, ignoring.");
      Serial.println("[CAUTION] invalid input from ground station, ignoring.");
      break;
  }
  return;
}//end function process_main_menu()

/*---------------------------------------------------------------------------------------------*/
// Get XBee RSSI:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Query the connected XBee radio for received signal strength (RSSI).
 *
 * This function places the XBee module into command mode, issues the ATDB
 * command to request the RSSI of the last received packet, parses the hex
 * response into a decimal value, and prints the result over both Serial and
 * Xbee in dBm.
 *
 * @note The function uses blocking delays and loops; it will not return until
 *       the XBee responds correctly with "OK\r".
 *
 * @warning Assumes that Xbee is connected to an XBee radio and already
 *          initialized at the correct baud rate.
 *
 * @post After the function runs, the XBee is returned to data mode using the
 *       ATCN command.
 *
 * @see Digi XBee AT Command Reference for details on ATDB and ATCN commands.
 */
void get_sat_rssi() {
  Xbee.println(" standby for RSSI");
  
  // put the radio in command mode:
  bool not_done = true;
  String ok_response = "OK\r";   //The﻿response we expect.
  String response = String("");  //Create an empty string
  Serial.println("Starting get_sat_rssi()");
  
  // Read the text of the response into the response variable
  while (not_done) {  // As long as we did not get a response from the XBee
    response = String("");
    delay(1100);
    Xbee.print("+++");  // Put the XBee 3 into 'Command Mode'
    // Serial.print("+++");   // Put the XBee 3 into 'Command Mode'

    // delay(1100);  // Wait for the XBee to finish
    while (response.length() < ok_response.length()) {
      if (Xbee.available() > 0) {

        response += (char)Xbee.read();  // Read a single character at a time
      }
    }
    not_done = !response.equals(ok_response);  // Set the not_done flag to the opposite of the result of equality check
  }
  // Serial.println(response);
      

  // If we got the right response, configure the radio and return true.   
  Xbee.print("ATDB\r");  // destination high and destination low addresses set to 0 means all messages will only go
  delay(100);               // Wait for the XBee
  response = String("");
  while (Xbee.available() > 0) {
    response += (char)Xbee.read();  //Read a single character at a time
    
  }
  Serial.println(response);
  Xbee.print("ATCN\r");  // Switch back to data mode

  String response2 = response; 
  uint32_t dec_response = strtoul(response2.c_str(), NULL, 16);

  Xbee.print("RSSI: -");
  Xbee.print(dec_response);
  Xbee.println(" dBm");
  Serial.println("sat rssi sent");
} // end function get_rssi()

/*---------------------------------------------------------------------------------------------*/
// Get Battery Information:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Send MAX17048 battery telemetry over the XBee link.
 *
 * Reads:
 *  - Battery voltage (V)
 *  - State of Charge (SOC, %)
 *  - Change / discharge rate (dSOC/dt, %/hr; negative = discharging)
 *
 * Output format (single line):
 *   BAT,V=<volts>,SOC=<percent>,CR=<percent_per_hr>
 */
void send_battery_telemetry() {

  const float v = lipo.getVoltage();
  const float soc = lipo.getSOC();
  const float crate = lipo.getChangeRate(); // %/hr (positive=charging, negative=discharging)

  // Ground-station friendly, parseable response
  Xbee.print("BAT,V=");
  Xbee.print(v, 3);
  Xbee.print("V, SOC=");
  Xbee.print(soc, 1);
  Xbee.print("%, CR=");
  Xbee.print(crate, 3);
  Xbee.println("%/hr");

  // Also mirror to USB serial for debugging
  Serial.print("[BAT] V=");
  Serial.print(v, 3);
  Serial.print(" V, SOC=");
  Serial.print(soc, 1);
  Serial.print(" %, CR=");
  Serial.print(crate, 3);
  Serial.println(" %/hr");
} 


/*---------------------------------------------------------------------------------------------*/
// Manually Set Reaction Wheel Speed:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Prompts user to manually set the reaction wheel motor speed
 * 
 * Requests a throttle percentage from the user (-100 to 100) via Serial or XBee,
 * validates the input, and applies it to the motor driver.
 * 
 * @note Input values outside the range [-100, 100] are clamped to the limits.
 * 
 * @return none
 */
void manual_set_RW_speed(){
  Serial.println("Enter RW Motor Throttle Percent (-100 to 100):");
  Xbee.println("Enter RW Motor Throttle Percent (-100 to 100):");
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  
  int rw_speed_int = get_command_from_ground_station();
  if (rw_speed_int>100) rw_speed_int = 100;
  if (rw_speed_int<-100) rw_speed_int = -100;
  float rw_speed = float(rw_speed_int) / 100.0;
  
  Serial.println("Setting motor speed to: ");
  Serial.println(rw_speed);
  Xbee.println("Setting motor speed to: ");
  Xbee.println(rw_speed);

  delay(500);

  driver.setOutput(rw_speed);

  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer
}

/*---------------------------------------------------------------------------------------------*/
// Run Lab 6 Test:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Runs the test for Lab 6 - Attitude Determination
 * 
 * Collects IMU (gyroscope, magnetometer) and sun sensor data at regular intervals
 * and logs all readings to an SD card file. Test continues until user sends 'X'.
 * 
 * @note Data is written to a CSV file with the following columns:
 *       mcu time (ms), gyro Z (dps), mag X (uT), mag Y (uT), sun direction (deg),
 *       sun_plusX, sun_plusY, sun_minusX, sun_minusY
 * 
 * @return none
 */
void lab6_run_test() {

  char file_name[40];
  if(sd_createDataFile(&dataFile, "Lab6_test")){
    // write header row:
    dataFile.println("mcu time (ms),gyro Z (dps),mag X (uT),mag Y (uT),sun direction (deg),sun_plusX,sun_plusY,sun_minusX,sun_minusY");
    dataFile.flush();
    dataFile.getName(file_name, sizeof(file_name));
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 6 test, send any key to begin (send 'X' to stop test)...");
  Xbee.println("[INFO] Ready to start Lab 6 test, send any key to begin (send 'X' to stop test)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer
  
  timeNext_testPoint = millis();
  int test_point_count = 0;
  neopixelWrite(RGB_BUILTIN, 255, 165, 0); // Set to orange (R=255, G=165, B=0)
  while(true){

    if(Serial.available() > 0 || Xbee.available() > 0) { // Check for user input from USB or XBee
      char c = (Serial.available() > 0) ? Serial.read() : Xbee.read();
      if(c == 'X' || c == 'x') { // If user sent 'X', stop the test
        Serial.print("[INFO] Test Complete. File ");
        Serial.print(file_name);
        Xbee.print("[INFO] Test Complete. File: ");
        Xbee.print(file_name);
        if(dataFile.close()) {
          Serial.println(" closed.");
          Xbee.println(" closed.");
        } else {
          Serial.println(" failed to close.");
          Xbee.println(" failed to close.");
        }
        return;
      } else {
        Serial.println("[CAUTION] Invalid Input, continuing test...");
        Xbee.println("[CAUTION] Invalid Input, continuing test...");
      }
    }

    uint32_t timeNow = millis();
    if(timeNow > timeNext_testPoint){ // Collect Test Point loop
      test_point_count++;
      timeNext_testPoint += interval_testPoint; // Update time for next Test Point
      
      // Collect IMU Test Point:
      imu_sensor.getAGMT();
      gyro_Z = imu_sensor.gyrZ(); 
      mag_X = imu_sensor.magX();
      mag_Y = imu_sensor.magY();

      // Collect Sun Sensor Test Point:
      // Average readings for each analog channel
      sun_plusX = 0.0;
      sun_minusX = 0.0;
      sun_plusY = 0.0;
      sun_minusY = 0.0;
      for (int i = 0; i < n_sun_sensor_reads; i++) {
        sun_plusX += analogRead(SUN_SENSOR_PLUS_X_PIN);
        sun_minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
        sun_plusY += analogRead(SUN_SENSOR_PLUS_Y_PIN);
        sun_minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
      }
      sun_plusX /= n_sun_sensor_reads;
      sun_minusX /= n_sun_sensor_reads;
      sun_plusY /= n_sun_sensor_reads;
      sun_minusY /= n_sun_sensor_reads;

      // ////////////* find sun direction *////////////////////////////////////////
      // // // uncomment sun_plusX & sun_plusY lines to calculate sun direction
      // // // (highlight them, CTRL-/)
      S_mag = sun_plusX + sun_minusX + sun_plusY + sun_minusY;
      // sun_X = ;
      // sun_Y = ;
      sun_direction = (atan2(sun_Y, sun_X) * RAD_TO_DEG);
      if (sun_direction < 0) {
        sun_direction += 360; // Adjust to range 0-360
      }
      /////////////////////////////////////////////////////////////////////////////

      // Print data to .csv file:
      dataFile.print(timeNow);
      dataFile.print(",");
      dataFile.print(gyro_Z,4);
      dataFile.print(",");
      dataFile.print(mag_X,4);
      dataFile.print(",");
      dataFile.print(mag_Y,4);
      dataFile.print(",");
      dataFile.print(sun_direction,4);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.println(sun_minusY);

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += timeNow;
      test_point_string += ",gZ:";
      test_point_string += gyro_Z;
      test_point_string += ",mX:";
      test_point_string += mag_X;
      test_point_string += ",mY:";
      test_point_string += mag_Y;
      test_point_string += ",sDir:";
      test_point_string += sun_direction;
      test_point_string += ",s+X:";
      test_point_string += sun_plusX;
      test_point_string += ",s+Y:";
      test_point_string += sun_plusY;
      test_point_string += ",s-X:";
      test_point_string += sun_minusX;
      test_point_string += ",s-Y:";
      test_point_string += sun_minusY;
      test_point_string += "\n";

      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      if(test_point_count % 10 == 0){ 
        dataFile.flush(); // save file every 10 test points
        Xbee.print("tp:");
        Xbee.println(test_point_count);
        // Xbee.print(test_point_string);
      }
    }
  }  
}

/*---------------------------------------------------------------------------------------------*/
// Lab 7: Run Test A
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Runs Test A - Tabletop static test. Initiates a step input at 100% speed 3 seconds into test. 
 *         Used to determine reaction wheel torque. Test duration is 15 seconds.
 * @details
 *   - 0-3s: Motor off (speed_pwm = 0)
 *   - 3-10s: Motor at full speed (speed_pwm = 1.0)
 *   - 10-15s: Motor off (speed_pwm = 0)
 * @return void
 */
void lab7_run_test_A() {
    
  if(sd_createDataFile(&dataFile, "Lab7_testA")){
    // write header row:
    dataFile.println("mcu time(ms),gyro_Z(deg/s),mag_X(uT),mag_Y(uT),sun_direction(deg),sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),w_RW_cmd(RPM),w_RW_meas(RPM)");
    dataFile.flush();
    char file_name[40];
    dataFile.getName(file_name, sizeof(file_name));
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 7 test A, send any key to begin (wait for test to complete or send 'X' to abort)...");
  Xbee.println("[INFO] Ready to start Lab 7 test A, send any key to begin (wait for test to complete or send 'X' to abort)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer

  timeNext_testPoint = millis();
  bool newUserInput = false;
  float speed_pwm = 0.0;
  uint32_t t0 = millis();
  neopixelWrite(RGB_BUILTIN, 0, 255, 255); // Set to cyan (R=0, G=255, B=255)
  while (true) { //test loop
    
    // Check for User Input:
    char c;
    if (Serial.available() > 0) {  // Check for user input from USB
      c = Serial.read();
      newUserInput = true;
    }
    if (Xbee.available() > 0) {  // Check for user input from USB
      c = Xbee.read();
      newUserInput = true;
    }
    if(newUserInput){
      newUserInput = false;
      switch (c) {
        case 'X':
        case 'x':
          dataFile.close();
          Serial.print("[CAUTION] Test Canceled Early. File closed.");
          Xbee.print("[CAUTION] Test Canceled Early. File closed.");
          driver.setOutput(0);
          return;
        default:
          Serial.println("[CAUTION] Invalid Input, continuing test...");
          Xbee.println("[CAUTION] Invalid Input, continuing test...");
          break;
      }
    }

    // Set RW Motor Speed:
    if ((millis() - t0) > 3000 && (millis() - t0) < 10000){
      speed_pwm = 1.0;
      driver.setOutput(speed_pwm);
    } else if (millis() - t0 > 10000) {
      speed_pwm = 0.0;
      driver.setOutput(speed_pwm);
    }

    // Record test point:
    if (millis() > timeNext_testPoint) {          // Collect Test Point loop
      timeNext_testPoint += interval_testPoint;  // Update time for next Test Point
      uint32_t time = millis() - t0;

      // Get commanded reaction wheel speed:
      float w_RW_cmd = -speed_pwm * 1000.0 * MOTOR_VOLTAGE / 12.0;
      // Get measured reaction wheel speed:
      static int64_t lastCount = 0;
      static uint32_t timeLastEncMeas = 0;
      uint32_t timeNow = millis();
      int64_t c = enc.getCount();
      int64_t dc = c - lastCount;
      float dt_s = (timeNow - timeLastEncMeas) / 1000.0f;
      float rev = (float)dc / ((float)CPR * 10.0);
      float w_RW_meas = (rev / dt_s) * 60.0f;
      lastCount = c;
      timeLastEncMeas = timeNow;

      // Print data to .csv file:
      dataFile.print(time);
      dataFile.print(",");
      dataFile.print(gyro_Z);
      dataFile.print(",");
      dataFile.print(mag_X);
      dataFile.print(",");
      dataFile.print(mag_Y);
      dataFile.print(",");
      dataFile.print(sun_direction);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.print(sun_minusY);
      dataFile.print(",");
      dataFile.print(w_RW_cmd);
      dataFile.print(",");
      dataFile.println(w_RW_meas);

      dataFile.flush();  // save file

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += time;
      test_point_string += ",w_RW_cmd:";
      test_point_string += w_RW_cmd;
      test_point_string += ",w_RW_meas:";
      test_point_string += w_RW_meas;

      test_point_string += "\n";
      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      // Xbee.print(test_point_string);

      // Serial.println(millis() - time + t0);
    }

    // End test if complete:
    if (millis() - t0 > 15000){
      dataFile.close();
      Serial.print("[INFO] Test A Complete. File closed.");
      Xbee.print("[INFO] Test A Complete. File closed.");
      return;
    }
  }
}  // end function lab7_run_test_A()

/*---------------------------------------------------------------------------------------------*/
// Lab 7: Run Test B
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Runs Lab 7 Test B - Reaction wheel control and sensor data collection test.
 * 
 * This function performs a comprehensive test of the spacecraft's attitude determination
 * and control system by collecting sensor data while commanding a reaction wheel motor.
 * The test runs for 40 seconds and logs all measurements to an SD card file.
 * The reaction wheel is commanded with a speed profile defined in set_speed_test_B().
 * 
 * @return void
 * 
 * @note
 * - Test duration is fixed at 40 seconds
 * - Test points are collected at intervals defined by interval_testPoint
 * - XBee data transmission is decimated (every 10th test point)
 * - Motor command speed is calculated as: -speed_pwm * 1000 * MOTOR_VOLTAGE / 12
 * - Sun direction is calculated from 4-quadrant sun sensor using atan2 function
 * - Data is flushed to SD card every 10 test points for data safety
 * 
 * @see set_speed_test_B()
 * @see sd_createDataFile()
 * @see IMU sensor getAGMT(), gyrZ(), magX(), magY() methods
 * @see Encoder getCount() method
 */
void lab7_run_test_B()
{
  if(sd_createDataFile(&dataFile, "Lab7_testB")){
    // write header row:
    dataFile.println("mcu time(ms),gyro_Z(deg/s),mag_X(uT),mag_Y(uT),sun_direction(deg),sun_plusX(count),sun_plusY(count),sun_minusX(count),sun_minusY(count),w_RW_cmd(RPM),w_RW_meas(RPM)");
    dataFile.flush();
    char file_name[40];
    dataFile.getName(file_name, sizeof(file_name));
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 7 test B, send any key to begin (wait for test to complete or send 'X' to abort)...");
  Xbee.println("[INFO] Ready to start Lab 7 test B, send any key to begin (wait for test to complete or send 'X' to abort)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer

  neopixelWrite(RGB_BUILTIN, 255, 0, 255); // Set to magenta (R=255, G=0, B=255)
  static uint32_t t0;
  t0 = millis();
  timeNext_testPoint = millis();
  while (true) { // test loop
    
    // Check for User Input:
    char c;
    bool newUserInput = false;
    if (Serial.available() > 0) {  // Check for user input from USB
      c = Serial.read();
      newUserInput = true;
    }
    if (Xbee.available() > 0) {  // Check for user input from USB
      c = Xbee.read();
      newUserInput = true;
    }
    if(newUserInput){
      newUserInput = false;
      switch (c) {
        case 'X':
        case 'x':
          dataFile.close();
          Serial.print("[CAUTION] Test Canceled Early. File closed.");
          Xbee.print("[CAUTION] Test Canceled Early. File closed.");
          driver.setOutput(0);
          return;
        default:
          Serial.println("[CAUTION] Invalid Input, continuing test...");
          Xbee.println("[CAUTION] Invalid Input, continuing test...");
          break;
      }
    }

    

    // Record test point:
    int test_point_count = 0;
    if (millis() > timeNext_testPoint) {         // Collect Test Point loop
      timeNext_testPoint += interval_testPoint;  // Update time for next Test Point
      test_point_count++;
      uint32_t time = millis() - t0;

      // Set RW Motor Speed:
      float speed_pwm = set_speed_test_B(t0); 

      // Collect IMU Test Point:
      imu_sensor.getAGMT();
      gyro_Z = imu_sensor.gyrZ(); 
      mag_X = imu_sensor.magX();
      mag_Y = imu_sensor.magY();

      // Collect Sun Sensor Test Point:
      // Average readings for each analog channel
      sun_plusX = 0.0;
      sun_minusX = 0.0;
      sun_plusY = 0.0;
      sun_minusY = 0.0;
      for (int i = 0; i < n_sun_sensor_reads; i++) {
        sun_plusX += analogRead(SUN_SENSOR_PLUS_X_PIN);
        sun_minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
        sun_plusY += analogRead(SUN_SENSOR_PLUS_Y_PIN);
        sun_minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
      }
      sun_plusX /= n_sun_sensor_reads;
      sun_minusX /= n_sun_sensor_reads;
      sun_plusY /= n_sun_sensor_reads;
      sun_minusY /= n_sun_sensor_reads;

      // ////////////* find sun direction *////////////////////////////////////////
      // // // uncomment sun_plusX & sun_plusY lines to calculate sun direction
      // // // (highlight them, CTRL-/)
      S_mag = sun_plusX + sun_minusX + sun_plusY + sun_minusY;
      sun_X = (sun_plusX - sun_minusX) / S_mag;
      sun_Y = (sun_plusY - sun_minusY) / S_mag;
      sun_direction = (atan2(sun_Y, sun_X) * RAD_TO_DEG);
      if (sun_direction < 0) {
        sun_direction += 360; // Adjust to range 0-360
      }
      /////////////////////////////////////////////////////////////////////////////

      // Get commanded reaction wheel speed:
      float w_RW_cmd = -speed_pwm * 1000.0 * MOTOR_VOLTAGE / 12.0;
      // Get measured reaction wheel speed:
      static int64_t lastCount = 0;
      static uint32_t timeLastEncMeas = 0;
      uint32_t timeNow = millis();
      int64_t c = enc.getCount();
      int64_t dc = c - lastCount;
      float dt_s = (timeNow - timeLastEncMeas) / 1000.0f;
      float rev = (float)dc / ((float)CPR * 10.0);
      float w_RW_meas = (rev / dt_s) * 60.0f;
      lastCount = c;
      timeLastEncMeas = timeNow;

      // Print data to .csv file:
      dataFile.print(time);
      dataFile.print(",");
      dataFile.print(gyro_Z);
      dataFile.print(",");
      dataFile.print(mag_X);
      dataFile.print(",");
      dataFile.print(mag_Y);
      dataFile.print(",");
      dataFile.print(sun_direction);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.print(sun_minusY);
      dataFile.print(",");
      dataFile.print(w_RW_cmd);
      dataFile.print(",");
      dataFile.println(w_RW_meas);

      dataFile.flush();  // save file

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += time;
      test_point_string += ",gyro_Z:";
      test_point_string += gyro_Z;
      test_point_string += ",mag_X:";
      test_point_string += mag_X;
      test_point_string += ",mag_Y:";
      test_point_string += mag_Y;
      test_point_string += ",sun_direction:";
      test_point_string += sun_direction;
      test_point_string += ",sun_plusX:";
      test_point_string += sun_plusX;
      test_point_string += ",sun_plusY:";
      test_point_string += sun_plusY;
      test_point_string += ",sun_minusX:";
      test_point_string += sun_minusX;
      test_point_string += ",sun_minusY:";
      test_point_string += sun_minusY;
      test_point_string += ",w_RW_cmd:";
      test_point_string += w_RW_cmd;
      test_point_string += ",w_RW_meas:";
      test_point_string += w_RW_meas;

      test_point_string += "\n";
      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      if(test_point_count % 10 == 0){ 
        dataFile.flush(); // save file every 10 test points
        Xbee.print("tp:");
        Xbee.println(test_point_count);
        // Xbee.print(test_point_string);
      }
      // Serial.println(millis() - time + t0);
    }

    // End test if complete:
    if (millis() - t0 > 40e3){
      dataFile.close();
      Serial.print("[INFO] Test B Complete. File closed.");
      Xbee.print("[INFO] Test B Complete. File closed.");
      return;
    }
  }
} // end function lab7_run_test_B()

/*---------------------------------------------------------------------------------------------*/
// Set Wheel Speed:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Sets the reaction wheel motor speed according to a prescribed test profile for Lab 7 Test B.
 * 
 * Implements a dynamic speed profile over 50 seconds:
 *  - 0-10s: Hold at base speed (0.6)
 *  - 10-15s: Hold at base speed (0.6)
 *  - 15-17.5s: Ramp up to base + ramp speed
 *  - 17.5-20s: Ramp back down to base speed
 *  - 20-25s: Hold at base speed
 *  - 25-27.5s: Ramp down to 0
 *  - 27.5-30s: Ramp back up to base speed
 *  - 30-35s: Hold at base speed
 *  - 35s+: Turn off wheel
 * 
 * @return (float) commanded PWM motor speed (0.0 to 1.0)
 */
float set_speed_test_B(uint32_t t0) {
  static uint32_t elapsed; 
  static uint32_t t;
  static float base_speed = 0.6;
  static float ramp_speed = 0.4;
  float motor_PWM_cmd = 0.0;

  t = millis() - t0;

  if (t < 10e3) {  // hold still at half speed (10 sec)
    motor_PWM_cmd = base_speed;
    neopixelWrite(RGB_BUILTIN, 255, 255, 0); // Set to yellow (R=255, G=255, B=0)
  }

  else if (t < 15e3) {  // hold still at half speed (5 sec)
    motor_PWM_cmd = base_speed;
    neopixelWrite(RGB_BUILTIN, 0, 0, 255); // Set to blue (R=0, G=0, B=255)
  }

  else if (t < 17.5e3) {  // ramp up (2.5 sec)
    elapsed = t - 15e3;
    motor_PWM_cmd = base_speed + ramp_speed * elapsed / 2.5e3;
  }

  else if (t < 20e3) {  // ramp back down to half speed (2.5 sec)
    elapsed = t - 17.5e3;
    motor_PWM_cmd = base_speed + ramp_speed - (ramp_speed * elapsed / 2.5e3);
  }

  else if (t < 25e3) {  // hold new position (5 sec)
    motor_PWM_cmd = base_speed;
  }

  else if (t < 27.5e3) {  // ramp down (2.5 sec)
    elapsed = t - 25e3;
    motor_PWM_cmd = base_speed - (ramp_speed * elapsed / 2.5e3);

  } else if (t < 30e3) {  // ramp back up to half speed  (5 sec)
    elapsed = t - 27.5e3;
    motor_PWM_cmd = base_speed - ramp_speed + (ramp_speed * elapsed / 2.5e3);
  }

  else if (t < 35e3) {  // hold new position (5 sec)
    motor_PWM_cmd = base_speed;
    neopixelWrite(RGB_BUILTIN, 255, 0, 255); // Set to magenta (R=255, G=0, B=255)

  }

  else if (t > 35e3) {  // turn off wheel
    motor_PWM_cmd = 0;
    driver.setOutput(0);
    neopixelWrite(RGB_BUILTIN, 255, 255, 255); // Set to white (R=255, G=255, B=255)

  }

  driver.setOutput(motor_PWM_cmd);
  Serial.println(motor_PWM_cmd);

  return motor_PWM_cmd;
}  // end set_speed()


void stream_RWspeed()
{
  Serial.println("Ready to stream RW Motor speed, send any key to start. Send 'X' to stop.");
  Xbee.println("Ready to stream RW Motor speed, send any key to start. Send 'X' to stop.");
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received

  while(true){
    // Check for User Input:
    char c;
    bool newUserInput = false;
    if (Serial.available() > 0) {  // Check for user input from USB
      c = Serial.read();
      newUserInput = true;
    }
    if (Xbee.available() > 0) {  // Check for user input from USB
      c = Xbee.read();
      newUserInput = true;
    }
    if(newUserInput){
      newUserInput = false;
      switch (c) {
        case 'X':
        case 'x':
          return;
      }
    }

    static uint32_t timeLastEncMeas = millis();
    static int64_t lastCount = 0;
    #define ENC_SAMPLE_MS 50
    
    uint32_t now = millis();
    if (now - timeLastEncMeas >= ENC_SAMPLE_MS) {
      int64_t c = enc.getCount();
      int64_t dc = c - lastCount;

      float dt_s = (now - timeLastEncMeas) / 1000.0f;

      // If you use full-quad (x4), make sure CPR reflects *counts per rev after decoding*
      float rev = (float)dc / ((float)CPR * 10.0);
      float rpm = (rev / dt_s) * 60.0f;

      Serial.printf("count:%lld,dc:%lld,rpm:%.2f\n", (long long)c, (long long)dc, rpm);
      Xbee.printf("count:%lld,dc:%lld,rpm:%.2f\n", (long long)c, (long long)dc, rpm);

      lastCount = c;
      timeLastEncMeas = now;
    }
  }
} //end stream_RWspeed()



/////////////////////////////////////////////////////////////////////////////////////////////////
// LAB 8: THRUSTERS
/////////////////////////////////////////////////////////////////////////////////////////////////

static const char *LAB8_HEADER = "mcu time(ms),gyro_Z(deg/s),mag_X(uT),mag_Y(uT),w_RW_cmd(RPM),w_RW_meas(RPM),fan_plusZ_cmd(frac),fan_minusZ_cmd(frac),phase";

/*---------------------------------------------------------------------------------------------*/
// Lab 8 Helpers:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Sends a message to both USB Serial and XBee.
 */
static void lab8_println(const char *msg) {
  Serial.println(msg);
  Xbee.println(msg);
}

/**
 * @brief Blocks until the user sends any key, then clears both input buffers.
 */
static void lab8_wait_for_key() {
  while(!Serial.available() && !Xbee.available()){}
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read();
  while(Xbee.available()) Xbee.read();
}

/**
 * @brief Prompts the user for an integer and waits for the reply.
 *
 * @param msg prompt text
 * @return integer received (0 if the reply was not a number)
 */
static int lab8_prompt_int(const char *msg) {
  lab8_println(msg);
  while(!Serial.available() && !Xbee.available()){}
  delay(100);
  int value = get_command_from_ground_station();
  while(Serial.available()) Serial.read();
  while(Xbee.available()) Xbee.read();
  return value;
}

/**
 * @brief Returns true if the user sent 'X' or 'x' (abort). Other characters are ignored.
 */
static bool lab8_abort_requested() {
  char c = 0;
  if (Serial.available() > 0) c = Serial.read();
  if (Xbee.available() > 0) c = Xbee.read();
  return (c == 'X' || c == 'x');
}

/**
 * @brief Creates a Lab 8 data file on SD and writes the header row.
 *
 * @param preamble file name prefix (max 12 characters)
 * @return true if the file was created
 */
static bool lab8_open_file(const char *preamble) {
  if (!sd_createDataFile(&dataFile, preamble)) {
    lab8_println("[ERROR] Failed to create data file. Aborting test.");
    return false;
  }
  dataFile.println(LAB8_HEADER);
  dataFile.flush();
  char file_name[40];
  dataFile.getName(file_name, sizeof(file_name));
  Xbee.print("[INFO] Data file created successfully: ");
  Xbee.println(file_name);
  Serial.print("[INFO] Data file created successfully: ");
  Serial.println(file_name);
  return true;
}

/**
 * @brief Stops all actuators and closes the data file.
 *
 * @param msg message to send to the ground station
 */
static void lab8_end_test(const char *msg) {
  thrusters_off();
  driver.setOutput(0);
  dataFile.close();
  lab8_println(msg);
}

/**
 * @brief Reads the IMU into the gyro_Z, mag_X and mag_Y globals.
 */
static void lab8_read_imu() {
  imu_sensor.getAGMT();
  gyro_Z = imu_sensor.gyrZ();
  mag_X = imu_sensor.magX();
  mag_Y = imu_sensor.magY();
}

/**
 * @brief Measures reaction wheel speed from the encoder since the last call.
 *
 * @return wheel speed (RPM, gearbox output shaft)
 */
static float lab8_measure_RW_rpm() {
  static int64_t lastCount = 0;
  static uint32_t timeLastEncMeas = 0;
  uint32_t timeNow = millis();
  int64_t c = enc.getCount();
  float dt_s = (timeNow - timeLastEncMeas) / 1000.0f;
  float rpm = 0.0;
  if (dt_s > 0) {
    float rev = (float)(c - lastCount) / ((float)CPR * 10.0);
    rpm = (rev / dt_s) * 60.0f;
  }
  lastCount = c;
  timeLastEncMeas = timeNow;
  return rpm;
}

/**
 * @brief Reads sensors and writes one row to the data file and USB Serial.
 *
 * @param time test time (ms since test start)
 * @param rw_pwm commanded reaction wheel output (-1.0 to 1.0)
 * @param phase test phase number (meaning depends on the test)
 */
static void lab8_log_row(uint32_t time, float rw_pwm, int phase) {
  lab8_read_imu();
  float w_RW_cmd = -rw_pwm * 1000.0 * MOTOR_VOLTAGE / 12.0; // same scaling as Lab 7
  float w_RW_meas = lab8_measure_RW_rpm();

  char row[160];
  snprintf(row, sizeof(row), "%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.3f,%.3f,%d",
           (unsigned long)time, gyro_Z, mag_X, mag_Y, w_RW_cmd, w_RW_meas,
           thrusters_get_plusZ(), thrusters_get_minusZ(), phase);
  dataFile.println(row);
  dataFile.flush();
  Serial.println(row);
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8: Manually Set Thrusters
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Prompts for a thruster command and applies it until changed or stopped (cmd 0).
 *
 * Positive percent runs the +Z fan, negative runs the -Z fan. Use this with cmd 9 or a
 * gyro stream to check that each fan turns KestrelSAT the expected way.
 */
void manual_set_thrusters() {
  int pct = lab8_prompt_int("Enter Thruster Percent (-100 to 100, + = +Z fan, - = -Z fan):");
  pct = constrain(pct, -100, 100);
  thrusters_set(pct / 100.0);
  Serial.print("[INFO] Thrusters set to (%): ");
  Serial.println(pct);
  Xbee.print("[INFO] Thrusters set to (%): ");
  Xbee.println(pct);
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8: Thrust Sweep
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Bench test that steps each fan through 25/50/75/100% duty to measure thrust vs. command.
 *
 * Mount one fan at a time on a scale (or the thruster module on a pendulum) and record the
 * reading for each step. Each step is held 5 s with 3 s off between steps; the +Z fan runs
 * first, then the -Z fan. Phase column = step number (0 = off).
 */
void lab8_thrust_sweep() {
  const float steps[] = {0.25, 0.50, 0.75, 1.00};
  const int n_steps = sizeof(steps) / sizeof(steps[0]);
  const uint32_t STEP_ON_MS = 5000;
  const uint32_t STEP_OFF_MS = 3000;
  const uint32_t STEP_MS = STEP_ON_MS + STEP_OFF_MS;
  const uint32_t TEST_MS = 2 * n_steps * STEP_MS;

  if (!lab8_open_file("Lab8_sweep")) return;
  lab8_println("[INFO] Ready to start thrust sweep, send any key to begin (send 'X' to abort)...");
  lab8_wait_for_key();

  neopixelWrite(RGB_BUILTIN, 255, 255, 0); // Set to yellow (R=255, G=255, B=0)
  uint32_t t0 = millis();
  timeNext_testPoint = t0;
  int last_step = -1;
  while (true) {
    if (lab8_abort_requested()) {
      lab8_end_test("[CAUTION] Test Canceled Early. File closed.");
      return;
    }

    uint32_t t = millis() - t0;
    if (t >= TEST_MS) {
      lab8_end_test("[INFO] Thrust Sweep Complete. File closed.");
      return;
    }

    // Set fan duty for the current step:
    int step = t / STEP_MS;                     // 0..(2*n_steps - 1)
    bool on = (t % STEP_MS) < STEP_ON_MS;
    float duty = on ? steps[step % n_steps] : 0.0;
    if (step < n_steps) {
      thrusters_set_each(duty, 0.0);
    } else {
      thrusters_set_each(0.0, duty);
    }
    int phase = on ? step + 1 : 0;
    if (on && step != last_step) {
      last_step = step;
      char msg[64];
      snprintf(msg, sizeof(msg), "[INFO] %s fan at %d%%", step < n_steps ? "+Z" : "-Z", (int)(duty * 100));
      lab8_println(msg);
    }

    if (millis() > timeNext_testPoint) {
      timeNext_testPoint += interval_testPoint;
      lab8_log_row(t, 0.0, phase);
    }
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8: Open-Loop Thruster Slew
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Open-loop rest-to-rest slew using the thrusters only (KestrelSAT hung by a string).
 *
 * Profile (phase column):
 *  - 1: 0-2 s idle (KestrelSAT should be released and still)
 *  - 2: +Z fan at 100% for the burn time
 *  - 3: -Z fan at 100% for the same burn time to stop the rotation
 *  - 4: coast with thrusters off for 10 s
 *
 * With equal fans, the slew angle grows with the square of the burn time. Use the Simulink
 * model or thrust sweep data to choose the burn time for a 180 deg slew.
 */
void lab8_thruster_slew() {
  int burn_s = lab8_prompt_int("Enter burn time per direction in seconds (1 to 60):");
  burn_s = constrain(burn_s, 1, 60);
  const uint32_t IDLE_MS = 2000;
  const uint32_t BURN_MS = (uint32_t)burn_s * 1000;
  const uint32_t COAST_MS = 10000;

  if (!lab8_open_file("Lab8_slew")) return;
  lab8_println("[INFO] Hang and steady KestrelSAT, then send any key to begin (send 'X' to abort)...");
  lab8_wait_for_key();

  neopixelWrite(RGB_BUILTIN, 255, 255, 255); // Set to white (R=255, G=255, B=255)
  uint32_t t0 = millis();
  timeNext_testPoint = t0;
  while (true) {
    if (lab8_abort_requested()) {
      lab8_end_test("[CAUTION] Test Canceled Early. File closed.");
      return;
    }

    uint32_t t = millis() - t0;
    int phase;
    if (t < IDLE_MS) {
      phase = 1;
      thrusters_off();
    } else if (t < IDLE_MS + BURN_MS) {
      phase = 2;
      thrusters_set(1.0);
    } else if (t < IDLE_MS + 2 * BURN_MS) {
      phase = 3;
      thrusters_set(-1.0);
    } else if (t < IDLE_MS + 2 * BURN_MS + COAST_MS) {
      phase = 4;
      thrusters_off();
    } else {
      lab8_end_test("[INFO] Thruster Slew Complete. File closed.");
      return;
    }

    if (millis() > timeNext_testPoint) {
      timeNext_testPoint += interval_testPoint;
      lab8_log_row(t, 0.0, phase);
    }
  }
}

/*---------------------------------------------------------------------------------------------*/
// Lab 8: Reaction Wheel Momentum Dump
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Spins the RW to full speed, then slows it to zero while the thrusters hold the body still.
 *
 * Profile (phase column):
 *  - 1: 0-10 s hold KestrelSAT by hand while the RW spins up to 100% (LED white)
 *  - 2: 10-13 s release KestrelSAT when the LED turns blue; RW holds full speed
 *  - 3: RW command ramps from 100% to 0 over the ramp time; thrusters damp body rate
 *  - 4: 15 s more of rate damping with the RW off
 *
 * Slowing the wheel moves its momentum into the body. The thrusters fire against the measured
 * body rate (gyro Z) to remove that momentum: full thrust at DUMP_FULL_RATE_DPS, nothing inside
 * DUMP_DEADBAND_DPS. This needs the +Z fan to really turn KestrelSAT +Z (check with cmd 10 first).
 */
void lab8_momentum_dump() {
  const float DUMP_DEADBAND_DPS = 2.0;
  const float DUMP_FULL_RATE_DPS = 20.0;
  int ramp_s = lab8_prompt_int("Enter RW ramp-down time in seconds (5 to 900):");
  ramp_s = constrain(ramp_s, 5, 900);
  const uint32_t SPINUP_MS = 10000;
  const uint32_t RELEASE_MS = 3000;
  const uint32_t RAMP_MS = (uint32_t)ramp_s * 1000;
  const uint32_t DAMP_MS = 15000;
  const uint32_t RAMP_START = SPINUP_MS + RELEASE_MS;

  if (!lab8_open_file("Lab8_dump")) return;
  lab8_println("[INFO] Hold KestrelSAT still, then send any key to begin. Release when LED turns BLUE (send 'X' to abort)...");
  lab8_wait_for_key();

  neopixelWrite(RGB_BUILTIN, 255, 255, 255); // Set to white (R=255, G=255, B=255)
  uint32_t t0 = millis();
  timeNext_testPoint = t0;
  bool released = false;
  while (true) {
    if (lab8_abort_requested()) {
      lab8_end_test("[CAUTION] Test Canceled Early. File closed.");
      return;
    }

    uint32_t t = millis() - t0;
    int phase;
    float rw_pwm;
    if (t < SPINUP_MS) {
      phase = 1;
      rw_pwm = 1.0;
    } else if (t < RAMP_START) {
      phase = 2;
      rw_pwm = 1.0;
      if (!released) {
        released = true;
        neopixelWrite(RGB_BUILTIN, 0, 0, 255); // Set to blue (R=0, G=0, B=255): release KestrelSAT
        lab8_println("[INFO] Release KestrelSAT now.");
      }
    } else if (t < RAMP_START + RAMP_MS) {
      phase = 3;
      rw_pwm = 1.0 - (float)(t - RAMP_START) / RAMP_MS;
    } else if (t < RAMP_START + RAMP_MS + DAMP_MS) {
      phase = 4;
      rw_pwm = 0.0;
    } else {
      lab8_end_test("[INFO] Momentum Dump Complete. File closed.");
      return;
    }
    driver.setOutput(rw_pwm);

    if (millis() > timeNext_testPoint) {
      timeNext_testPoint += interval_testPoint;

      // Rate damping with thrusters (phases 3 and 4 only), using the latest gyro reading:
      if (phase >= 3) {
        float rate = gyro_Z;
        float cmd = 0.0;
        if (fabs(rate) > DUMP_DEADBAND_DPS) {
          cmd = constrain(-rate / DUMP_FULL_RATE_DPS, -1.0f, 1.0f);
        }
        thrusters_set(cmd);
      }

      lab8_log_row(t, rw_pwm, phase);
    }
  }
}
