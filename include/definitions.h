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

// Heading hold (cmd 15): bang-bang thruster control on gyro-integrated heading.
// The controller predicts where KestrelSAT will stop if it brakes now and fires the fan that
// drives that predicted error to zero. Tune these from command 14 data.
#define HOLD_ALPHA_DPS2         1.0    // angular accel from one fan (deg/s^2); slope of gyro_Z while one fan is on (~1.0 from 2026-11-03 cmd 16 log, +Z fan)
#define HOLD_FAN_LAG_S          0.5    // fan spin-up delay (s); extra lead added to braking
#define HOLD_DEADBAND_ON_DEG    2.0    // fire a fan when predicted error exceeds this (deg)
#define HOLD_DEADBAND_OFF_DEG   0.5    // turn the fan off when predicted error falls below this (deg)
#define HOLD_BIAS_CAL_MS        3000   // time KestrelSAT is held still to measure gyro bias (ms)
#define HOLD_MIN_OFF_MS         500    // after a fan turns off, neither fan fires for this long (ms); lets it spin down

// RW momentum dump (cmd 13): full-power fan against the averaged body rate.
#define DUMP_RATE_TAU_S         0.5    // averaging time constant on gyro_Z (s); raw gyro is noisy (about +/-3 deg/s at rest)
#define DUMP_ON_DPS             2.0    // fire the opposing fan when |averaged rate| exceeds this (deg/s)
#define DUMP_OFF_DPS            0.5    // turn it off when the rate falls below this or reverses (deg/s)

// Closed-loop RW momentum dump (cmd 17): the RW slows only while the thrusters hold the magnetic heading.
#define CLDUMP_MAX_RAMP_PCT_S   0.2    // fastest RW spin-down, percent of full command per second
#define CLDUMP_HEADING_OK_DEG   1.0    // full-speed spin-down while |heading error| is below this (deg)
#define CLDUMP_HEADING_LIMIT_DEG 3.0   // spin-down pauses when |heading error| reaches this (deg)
#define CLDUMP_RATE_OK_DPS      0.3    // full-speed spin-down while |averaged body rate| is below this (deg/s)
#define CLDUMP_RATE_LIMIT_DPS   1.0    // spin-down pauses when |averaged body rate| reaches this (deg/s)
#define CLDUMP_FAN_ON_DEG       2.0    // fire a fan when the predicted heading error passes this (deg); pushes back to MAG_HOLD_CENTER_DEG
#define CLDUMP_DECEL_OK_RPMS    2.0    // full-speed spin-down while the measured wheel slows less than this (RPM/s)
#define CLDUMP_DECEL_LIMIT_RPMS 5.0    // spin-down pauses when the measured wheel slows this fast (RPM/s); 2026-11-03 dump log: fans kept up at ~5-7 RPM/s

// Wheel-held momentum dump (cmd 18): the RW holds the magnetic heading while one thruster drains it.
// Measured RW speed vs PWM command, from the 2026-11-03 Lab8_cldump_data002 log (0.1 -> ~270 RPM held;
// speed collapses below ~1.2%). The 100% point is the Lab 7 spec (410 RPM). Update from new logs.
#define RW_TABLE_N              7
static const float RW_TABLE_PWM[RW_TABLE_N] = {0.0,  0.0105, 0.0124, 0.0216, 0.038, 0.10,  1.0};
static const float RW_TABLE_RPM[RW_TABLE_N] = {0.0,  0.0,    57.0,   142.0,  231.0, 279.0, 410.0};
#define WHL_KP                  0.5    // wanted body accel per deg of heading error (deg/s^2 per deg)
#define WHL_KD                  1.2    // wanted body accel per deg/s of body rate (deg/s^2 per deg/s)
#define WHL_COUPLING            0.12   // body accel (deg/s^2) per RPM/s of RW speed change; flip the sign if the RW turns the sat the wrong way
#define WHL_MAX_ACCEL_RPMS      30.0   // cap on RW speed target change (RPM/s)
#define WHL_TRIM_GAIN           2.0    // RW speed loop: trim change per RPM of speed error per second (1/s)
#define WHL_TRIM_MAX_RPM        150.0  // RW speed loop trim limit (RPM)
#define WHL_FILTER_TAU_S        0.2    // averaging on gyro rate and measured RW speed (s)
#define WHL_DONE_RPM            60.0   // dump phase ends below this RW speed; the wheel stalls near here anyway
#define WHL_MAX_DUMP_MS         300000 // dump phase safety limit (ms)

// Magnetic heading hold (cmd 16): same controller as cmd 15, heading from the magnetometer.
#define MAG_OFFSET_X_UT         -50.52 // hard-iron offset of mag X (uT); avg of 2026-11-03 cmd 16 calibrations (-50.55, -50.47); recalibrate with cmd 16 C
#define MAG_OFFSET_Y_UT         -27.41 // hard-iron offset of mag Y (uT); avg of -27.52 and -27.30 (a third run gave -42.45, treated as an incomplete turn)
#define MAG_FILTER_TAU_S        0.2    // low-pass time constant on mag X/Y (s); larger = smoother but more lag
#define MAG_HEADING_SIGN        -1.0   // +1 if mag heading increases when gyro_Z is positive, -1 if it decreases (team MATLAB shows -1)
#define MAG_HOLD_BAND_DEG       5.0    // fans stay off while the predicted stop is within +/- this of the target (deg)
#define MAG_HOLD_CENTER_DEG     0.5    // once outside the band, push until the predicted stop is within this of the target or past it (deg)
