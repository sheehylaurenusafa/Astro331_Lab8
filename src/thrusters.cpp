/**
 * @file thrusters.cpp
 * @brief Fan thruster driver for KestrelSAT Lab 8.
 *
 * @details Two fans are mounted so that one torques KestrelSAT about +Z and the other about -Z.
 * Each fan is switched by a MOSFET driven with a PWM signal from the MCU. Fans only blow one
 * way, so a signed command is split between the two fans: a positive command runs the +Z fan,
 * a negative command runs the -Z fan.
 *
 * @see definitions.h for fan pins and PWM settings
 */
#include "thrusters.h"
#include "definitions.h"

static const uint8_t FAN_PLUS_Z_CHANNEL  = 4; // LEDC channels (only used with Arduino-ESP32 core 2.x)
static const uint8_t FAN_MINUS_Z_CHANNEL = 5;
static const uint32_t FAN_DUTY_MAX = (1UL << FAN_PWM_RES_BITS) - 1;

static float plusZ_cmd = 0.0;  // last commanded duty for +Z fan (0.0 to 1.0)
static float minusZ_cmd = 0.0; // last commanded duty for -Z fan (0.0 to 1.0)

/*---------------------------------------------------------------------------------------------*/
// Write duty to one fan:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Writes a duty fraction to one fan's PWM output.
 *
 * @param pin GPIO pin of the fan
 * @param channel LEDC channel of the fan (Arduino-ESP32 core 2.x only)
 * @param duty duty fraction (0.0 to 1.0); values below FAN_MIN_DUTY are written as 0
 */
static void write_fan(uint8_t pin, uint8_t channel, float duty) {
  if (duty < FAN_MIN_DUTY) duty = 0.0;
  uint32_t counts = (uint32_t)(duty * FAN_DUTY_MAX + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, counts);
#else
  (void)pin;
  ledcWrite(channel, counts);
#endif
}

/*---------------------------------------------------------------------------------------------*/
// Initialize Thrusters:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Configures PWM outputs for both fans and turns them off.
 */
void thrusters_init() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(FAN_PLUS_Z_PIN, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
  ledcAttach(FAN_MINUS_Z_PIN, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
#else
  ledcSetup(FAN_PLUS_Z_CHANNEL, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
  ledcSetup(FAN_MINUS_Z_CHANNEL, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
  ledcAttachPin(FAN_PLUS_Z_PIN, FAN_PLUS_Z_CHANNEL);
  ledcAttachPin(FAN_MINUS_Z_PIN, FAN_MINUS_Z_CHANNEL);
#endif
  thrusters_off();
}

/*---------------------------------------------------------------------------------------------*/
// Set Thrusters (signed):
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Commands a signed torque direction and magnitude.
 *
 * @param cmd -1.0 to 1.0. Positive runs the +Z fan, negative runs the -Z fan, the other fan is off.
 */
void thrusters_set(float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);
  if (cmd >= 0) {
    thrusters_set_each(cmd, 0.0);
  } else {
    thrusters_set_each(0.0, -cmd);
  }
}

/*---------------------------------------------------------------------------------------------*/
// Set Each Thruster:
/*---------------------------------------------------------------------------------------------*/
/**
 * @brief Commands each fan independently (used for the thrust sweep).
 *
 * @param plusZ duty for the +Z fan (0.0 to 1.0)
 * @param minusZ duty for the -Z fan (0.0 to 1.0)
 */
void thrusters_set_each(float plusZ, float minusZ) {
  plusZ_cmd = constrain(plusZ, 0.0f, 1.0f);
  minusZ_cmd = constrain(minusZ, 0.0f, 1.0f);
  write_fan(FAN_PLUS_Z_PIN, FAN_PLUS_Z_CHANNEL, plusZ_cmd);
  write_fan(FAN_MINUS_Z_PIN, FAN_MINUS_Z_CHANNEL, minusZ_cmd);
}

/**
 * @brief Turns both fans off.
 */
void thrusters_off() {
  thrusters_set_each(0.0, 0.0);
}

/** @return last commanded +Z fan duty (0.0 to 1.0) */
float thrusters_get_plusZ() { return plusZ_cmd; }

/** @return last commanded -Z fan duty (0.0 to 1.0) */
float thrusters_get_minusZ() { return minusZ_cmd; }
