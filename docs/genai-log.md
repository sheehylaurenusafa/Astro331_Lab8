# GenAI Usage Log

Required by Astro 331 GenAI Level 4 policy: record how GenAI contributed and how the team reviewed or revised it.

| Date | Tool | What it produced | Files | Team review / changes |
|---|---|---|---|---|
| 2026-10-05 | Claude Code | Drafted `CLAUDE.md` (agent guidance) from the existing firmware and the Lab 8 guidance deck | `CLAUDE.md`, `docs/genai-log.md` | _to fill in_ |
| 2026-10-07 | Claude Code | Thruster firmware: fan PWM driver, commands 10-13 (manual thrusters, thrust sweep, open-loop slew, RW momentum dump with rate damping); CLAUDE.md updates | `src/thrusters.cpp`, `include/thrusters.h`, `include/definitions.h`, `src/main.cpp`, `CLAUDE.md` | _to fill in_ |
| 2026-10-22 | Claude Code | Command 14: data log (time, gyro Z, mag X/Y, both fan on/off states, RW commanded and measured speed) with key-controlled fans during the test | `src/main.cpp`, `CLAUDE.md` | _to fill in_ |
| 2026-10-26 | Claude Code | Command 15: closed-loop heading hold (gyro bias calibration, integrated gyro heading, bang-bang thruster control on predicted stopping error); tuning constants in `definitions.h` | `src/main.cpp`, `include/definitions.h`, `CLAUDE.md` | _to fill in_ |
| 2026-10-30 | Claude Code | Command 16: closed-loop heading hold on magnetometer heading (optional hard-iron calibration spin, low-pass filtered mag heading, gyro rate for braking, same bang-bang controller as cmd 15, now shared in a helper) | `src/main.cpp`, `include/definitions.h`, `CLAUDE.md` | _to fill in_ |
| 2026-11-03 | Claude Code | Command 16: +/-5 deg band around the target where the fans stay off; outside it the fans push back to center (team chose "back to center") | `src/main.cpp`, `include/definitions.h` | _to fill in_ |
| 2026-11-03 | Claude Code | Fix for cmds 15/16 fans never stopping and both running: a fan now turns off once the predicted stop reaches or passes the target (not only inside a narrow window), and the fans cannot switch straight from one to the other (HOLD_MIN_OFF_MS pause) | `src/main.cpp`, `include/definitions.h` | _to fill in_ |
| 2026-11-03 | Claude Code | Saved the team's cmd 16 magnetometer calibration as default offsets (X = -50.52 uT, Y = -27.41 uT; the -42.45 uT Y run was left out as an outlier) | `include/definitions.h` | _to fill in_ |
| 2026-11-03 | Claude Code | Set HOLD_ALPHA_DPS2 to 1.0 deg/s^2, measured from the team's cmd 16 log (gyro_Z rose ~10 deg/s over ~9 s with the +Z fan on) | `include/definitions.h` | _to fill in_ |
