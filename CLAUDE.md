# CLAUDE.md

Guidance for AI coding agents working in this repository.

## Project

USAFA Astro 331 Lab 8, the KestrelSAT Capstone, by team Gullible (C2C Grayson Groth and C2C Lauren Sheehy).
The goal is to add two PC-fan thrusters to KestrelSAT for z-axis slews and reaction wheel (RW) momentum dumping.
This repo starts from the instructor's Lab 6/7 firmware (Lt Col Wyatt Harris, v2.0).

Team requirements (draft, frozen at the Preliminary Data Review, Lsn 23):
- Rotate about z with no more than 5 cm of linear motion.
- Dump 0.16 N·m·s of RW momentum (number still being checked against the RW spec).
- Thruster attachment is removable / changeable.
- 180° slew in 3 s or less.
- Stretch: closed-loop heading hold within 5° while hung by a string.

The course grades **only what is in this repo**, so commit test data, plots, CAD, sim models and docs here, not just code.
GenAI use must be documented (see "GenAI log" below).

## Build and flash

PlatformIO project (`platformio.ini`, env `esp32thing_plus`, Arduino framework on espressif32).

```
pio run                 # build
pio run -t upload       # flash
pio device monitor -b 115200
```

There are no unit tests. `pio run` compiling cleanly is the minimum check before committing firmware changes.
Hardware behavior can only be verified on the satellite by the team, so say what was and was not tested.

## Layout

- `src/main.cpp`: all firmware. Has setup, loop, the ground-station command menu, and the Lab 6/7 test routines.
- `src/sd_funcitons.cpp` + `include/sd_functions.h`: SD card helpers. The misspelled filename is intentional; leave it.
- `include/definitions.h`: pin assignments and hardware constants.
- `src/thrusters.cpp` + `include/thrusters.h`: fan thruster PWM driver (`thrusters_set(-1..1)`, `thrusters_set_each`, `thrusters_off`).
- `lib/INA238-master/`: vendored INA238 current-sensor library (unused so far).

## Hardware (from `definitions.h` and `main.cpp`)

- SparkFun Thing Plus ESP32 MCU.
- ICM-20948 IMU over I2C; the code uses `gyrZ()`, `magX()`, and `magY()`.
- MAX17048 fuel gauge (I2C 0x36).
- RW: TB9051FTG driver (`MOTOR_PWM_1_PIN` 33, `MOTOR_PWM_2_PIN` 15) and a quadrature encoder (pins 14/32, `CPR` 64). The speed math assumes a 10:1 gearbox (`CPR * 10.0`).
- XBee on `Serial2` (RX 16, TX 17) at 9600 baud. USB Serial runs at 115200.
- SD card over SPI (CS 5).
- Sun sensor on A0 to A3.
- Thrusters: two Pi-FAN LD3007MS fans (30 mm, 5 V, 0.20 A, 2-wire), each low-side switched by a logic-level N-MOSFET with a flyback diode. PWM on `FAN_PLUS_Z_PIN` 27 and `FAN_MINUS_Z_PIN` 4 (`src/thrusters.cpp`, LEDC at 100 Hz). Fans blow one way only: the +Z fan torques +Z, the -Z fan torques -Z. If a fan turns the sat the wrong way, swap the pin numbers in `definitions.h`.

## Firmware conventions

- **Command menu**: `process_main_menu()` switches on an integer read from USB or XBee. To add a feature, add a `case`, print it in **both** menu listings (the XBee and Serial blocks), and update the command table in the file header comment. Commands in use: 0 to 13, 98, 99 (10 to 13 are Lab 8 thruster commands).
- **Test routines** follow the `lab7_run_test_A()` pattern:
  1. `sd_createDataFile(&dataFile, "<prefix>")` (prefix up to 12 characters) and write a CSV header with units in parentheses.
  2. Wait for any key.
  3. Run a loop that aborts on `X`/`x`.
  4. Log every `interval_testPoint` (50 ms) using `millis() - t0` as the time column, then `flush()` after each row.
  5. Close the file and **command actuators to zero** on both normal exit and abort.
- Print user messages to both `Serial` and `Xbee`. Use the `[INFO]`, `[CAUTION]`, `[WARN]`, and `[ERROR]` prefixes.
- RGB LED colors show the mode (red = startup, green = idle, orange = Lab 6, cyan = Lab 7A, magenta = Lab 7B, yellow = Lab 8 sweep, white = Lab 8 slew / dump hold, blue = Lab 8 dump release cue). Give each new mode its own color.
- Lab 8 tests use the `lab8_*` helpers at the bottom of `main.cpp` (open file, log row, abort check, end test) and share one CSV format: time, gyro_Z, mag_X, mag_Y, RW cmd/meas RPM, both fan duties, phase.
- Keep the existing Doxygen-style `/** @brief ... */` comment blocks on functions.
- Put pins and hardware constants in `definitions.h`, not in `main.cpp`.

## Known gotchas

- `setup()` has `while (!Serial)`, but on this ESP32 board (USB-UART bridge) `Serial` is always true, so it does not block untethered boots (Lab 7 Test B ran hung over XBee).
- `setup()` halts forever if the IMU is not found.
- Lab 7 Test A logs `gyro_Z`, `mag_*`, and the sun values without reading the sensors inside its loop, so those columns are stale in that test.
- Test loops are blocking, and the main menu is not serviced during a test.

## Planned work

Planned work (see the team plan) covers:
- fan PWM control
- menu commands for a thrust sweep, an open-loop slew, and a momentum dump
- telemetry for the ground station
- a Simulink single-axis model
- a 3D-printed removable mount
- verification test data and plots for each requirement

Suggested folders: `sim/`, `cad/`, `data/`, `analysis/`, `docs/`.

## GenAI log

Course policy (GenAI Level 4) requires documenting how AI contributed and what the team changed.
When an agent makes a substantive change, add a dated entry to `docs/genai-log.md`. Each entry says what was generated and which files changed, and leaves room for the team's review notes.
