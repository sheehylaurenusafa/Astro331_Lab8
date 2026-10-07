/*---------------------------------------------------------------------------------------------*/
// Configuration:
/*---------------------------------------------------------------------------------------------*/
#define SD_CS_PIN 5  // Chip select pin for the microSD card on Thing Plus
// #define SD_CS_PIN 33  // Chip select pin for the microSD card on Feather Adalogger
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23

#define XBEE_RX 16  // ESP32 RX <- XBee DOUT
#define XBEE_TX 17  // ESP32 TX -> XBee DIN

#define SUN_SENSOR_PLUS_X_PIN   A0
#define SUN_SENSOR_MINUS_X_PIN  A2
#define SUN_SENSOR_PLUS_Y_PIN   A1
#define SUN_SENSOR_MINUS_Y_PIN  A3

#define MOTOR_VOLTAGE           5.05
#define CPR                     64
#define MOTOR_PWM_1_PIN         33
#define MOTOR_PWM_2_PIN         15
#define ENCODER_PIN_A           14
#define ENCODER_PIN_B           32



// Thrusters (Lab 8): two Pi-FAN LD3007MS 5V 0.20A 2-wire fans, each switched low-side by a
// logic-level N-MOSFET (gate <- GPIO, drain <- fan black wire, fan red wire <- 5V battery rail,
// flyback diode across the fan). If a fan turns KestrelSAT the wrong way, swap these two pins.
#define FAN_PLUS_Z_PIN          27     // fan that torques KestrelSAT in +Z (counter-clockwise viewed from above)
#define FAN_MINUS_Z_PIN         13     // fan that torques KestrelSAT in -Z (clockwise viewed from above); shares the STAT LED pin
#define FAN_PWM_FREQ_HZ         100    // low PWM frequency suits 2-wire brushless fans switched on their supply
#define FAN_PWM_RES_BITS        10     // PWM duty resolution (0-1023)
#define FAN_MIN_DUTY            0.0    // duty below which a fan stalls; set from thrust sweep (cmd 11) data
