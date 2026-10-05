# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

USAFA Astro 331 Lab 8 (KestrelSAT capstone), Team Gullible. The repo starts from the Lab 7 KestrelSAT firmware; the Lab 8 goal is to add two PC-fan thrusters (3D-printed removable mount) for open-loop slews, reaction wheel (RW) momentum dumping, and a stretch closed-loop heading hold. Everything graded must live in this repo, and GenAI use must be documented (what was generated and what the team changed).

## Build and upload (PlatformIO)

Single environment `esp32thing_plus` in `platformio.ini`:

```bash
pio run                      # build
pio run -t upload            # flash over USB
pio device monitor -b 115200 # USB serial console
```

There are no unit tests for the firmware (`test/` holds only the PlatformIO placeholder README). `lib/INA238-master/` is a vendored library that is not currently used by `src/`; its own tests and CI are upstream's, not this project's.

Board caveat: `platformio.ini` targets `sparkfun_esp32s2_thing_plus_c`, while the code comments and pin choices (SPI 18/19/23, XBee UART2 on 16/17) describe the ESP32-WROOM Thing Plus. Confirm the actual board before changing pins or the board setting.

## Architecture

All application logic is in `src/main.cpp` (Arduino `setup()`/`loop()`); `include/definitions.h` holds every pin and hardware constant; `src/sd_funcitons.cpp` (note the filename typo) + `include/sd_functions.h` wrap SdFat.

- **Command interface.** `loop()` polls every 10 ms. `process_main_menu()` reads one integer command line from either the XBee (`HardwareSerial Xbee(2)`, 9600 baud) or USB Serial (115200) and dispatches via a `switch`. Commands: 0 stop RW, 1 menu, 2 RSSI, 3 LED, 4 battery, 5 manual RW throttle, 6 Lab 6 test, 7 Lab 7 test A, 8 Lab 7 test B, 9 stream RW speed, 98/99 SD file list/print (USB only). Adding a mode means: a new `case`, a matching line in **both** the XBee and Serial menu printouts in `case 1`, a prototype in the prototypes block, and the doc table at the top of `main.cpp`.
- **Test functions are blocking loops.** Each `labN_run_test*()` creates a CSV via `sd_createDataFile(&dataFile, "<prefix>")` (auto-numbers `<prefix>_data###.csv`, prefix truncated to 12 chars), writes a header row, waits for any key, then loops until its time limit or an `x`/`X` abort from either link. Inside it samples every `interval_testPoint` (50 ms), logs a CSV row with `flush()`, and prints a `t:...,key:value` line to Serial (XBee output is commented out or decimated to save bandwidth). Nothing else runs while a test is active, so new thruster/controller modes should follow this same pattern. Always set actuators to zero on abort and at test end.
- **Sensors.** ICM-20948 IMU over I2C (`imu_sensor.getAGMT()` then `gyrZ()`, `magX()`, `magY()`); 4-channel analog sun sensor averaged over `n_sun_sensor_reads` with 12-bit ADC; MAX17048 fuel gauge (`lipo`). `setup()` halts forever if the IMU or SD card is missing.
- **Reaction wheel.** `TB9051FTGMotorCarrier driver` on `MOTOR_PWM_1_PIN`/`MOTOR_PWM_2_PIN` (33/15), `setOutput()` takes -1.0..1.0. Speed comes from `ESP32Encoder enc` (full quad on 14/32): RPM = Δcount / (CPR·10) / Δt · 60, where the factor 10 is the gearbox ratio. Commanded RPM is estimated as `-speed_pwm * 1000 * MOTOR_VOLTAGE / 12`.
- **Shared state is global.** `gyro_Z`, `mag_X`, `mag_Y`, sun values, `dataFile`, and timing variables are file-scope globals reused across tests. Only `lab6_run_test()` actually calls `getAGMT()` and reads the sun sensor; the Lab 7 tests log those columns but never refresh them, so their IMU/sun values are stale. Any new test that needs body rate must read the IMU itself.
- **Status LED.** The onboard WS2812 (`neopixelWrite(RGB_BUILTIN, r, g, b)`) signals mode: red startup, green idle, orange Lab 6, cyan Lab 7A, magenta Lab 7B.

## Free resources for thrusters

Pins already used (see `definitions.h`): SD CS 5, SPI 18/19/23, XBee 16/17, sun sensor A0-A3, RW PWM 33/15, encoder 14/32, I2C (default Wire pins). Fan drive pins need to be chosen from what remains and added to `definitions.h`.

## Project planning notes

Requirements, milestones (Proposal Lsn 19, Prelim Data Review Lsn 23, Final Lsn 31), and the build plan live outside the repo in the project's shared notes (`lab8-requirements-and-plan.md`).
