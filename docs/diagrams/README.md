# Code diagrams: commands 16 and 18

Slide diagrams of the Lab 8 firmware for the final presentation, drawn from `src/main.cpp`, `src/thrusters.cpp` and `include/definitions.h` on `claude/thruster-firmware` (commit 766ee90). Colors follow the team deck theme.

| File | Shows |
|---|---|
| `01-overview` | Command dispatch (`process_main_menu()` → `switch`) into cmd 16 / cmd 18, and the sense → decide → actuate → log step both commands share |
| `02-cmd16-mag-heading-hold` | `lab8_mag_heading_hold()`: optional mag calibration, gyro bias and target, hold loop, 'Z' / 'X' keys |
| `03-bang-bang-hold-fan-state` | `lab8_hold_fan_state()`: predicted stopping error and the fan on/off logic (used by cmd 16 and cmd 18 phase 4) |
| `04-cmd18-wheel-hold-dump-phases` | `lab8_wheel_hold_dump()`: phases 1–4 and the 50 ms tick |
| `05-cmd18-phase3-control-loop` | Cmd 18 phase 3: heading PD → wheel speed target → speed trim → PWM lookup, with the fan torque cancelled |
| `06-thruster-driver` | `thrusters.cpp`: `thrusters_set()` → `thrusters_set_each()` → `write_fan()` → LEDC PWM on GPIO 27 / 13 → MOSFET → fans |

Each diagram is a `.png` (150 dpi, for slides) and a `.svg`. The SVGs use the Inter and JetBrains Mono fonts; if those are not installed, use the PNGs.

## Regenerating

Sources are Graphviz files in `source/`, written by `source/gen.py`:

```
python3 source/gen.py source
for f in source/*.dot; do n=$(basename $f .dot)
  L=$(grep -q 'layout=neato' $f && echo "-Kneato -n2")
  dot $L -Tpng -Gdpi=150 $f -o $n.png; dot $L -Tsvg $f -o $n.svg
done
```

If constants in `definitions.h` change (gains, thresholds, offsets), update the labels in `gen.py` to match.
