# GenAI Usage Log

Required by Astro 331 GenAI Level 4 policy: record how GenAI contributed and how the team reviewed or revised it.

| Date | Tool | What it produced | Files | Team review / changes |
|---|---|---|---|---|
| 2026-10-05 | Claude Code | Drafted `CLAUDE.md` (agent guidance) from the existing firmware and the Lab 8 guidance deck | `CLAUDE.md`, `docs/genai-log.md` | _to fill in_ |
| 2026-10-07 | Claude Code | Thruster firmware: fan PWM driver, commands 10-13 (manual thrusters, thrust sweep, open-loop slew, RW momentum dump with rate damping); CLAUDE.md updates | `src/thrusters.cpp`, `include/thrusters.h`, `include/definitions.h`, `src/main.cpp`, `CLAUDE.md` | _to fill in_ |
| 2026-10-22 | Claude Code | Command 14: data log (time, gyro Z, mag X/Y, both fan on/off states, RW commanded and measured speed) with key-controlled fans during the test | `src/main.cpp`, `CLAUDE.md` | _to fill in_ |
| 2026-10-26 | Claude Code | Command 15: closed-loop heading hold (gyro bias calibration, integrated gyro heading, bang-bang thruster control on predicted stopping error); tuning constants in `definitions.h` | `src/main.cpp`, `include/definitions.h`, `CLAUDE.md` | _to fill in_ |
| 2026-10-30 | Claude Code | Command 16: closed-loop heading hold on magnetometer heading (optional hard-iron calibration spin, low-pass filtered mag heading, gyro rate for braking, same bang-bang controller as cmd 15, now shared in a helper) | `src/main.cpp`, `include/definitions.h`, `CLAUDE.md` | _to fill in_ |
