# EV Battery Thermal Management Controller (ESP32 + FreeRTOS)

A working Wokwi simulation of a multi-task battery thermal-management controller.
This replaces the earlier stalled attempt: the circuit is now built only from
parts that exist in Wokwi, the firmware has no blocking `delay()` calls in task
loops, and every output is observable (two LEDs, a buzzer, and a CSV serial
stream) so there is nothing to guess about.

## What it does

| Task | Core | Period | Responsibility |
| --- | --- | --- | --- |
| `sensor` | 0 | 100 ms | Reads the NTC thermistor (median of 5), publishes the newest sample |
| `control` | 1 | 50 ms | Hysteresis + proportional fan, pump staging, over-temperature latch |
| `telemetry` | 1 | 500 ms | Prints `ms,temp_c,fan_pct,pump_pct,fault` to the serial monitor |
| `input` | 0 | 10 ms | 30 ms debounced fault-acknowledge button |

Concurrency primitives used: a **queue of length 1** with `xQueueOverwrite` for
the latest sensor sample, a **mutex** around the shared `SystemState`, and
`vTaskDelayUntil` for fixed-period tasks. The control task owns the outputs, so
there is no interleaving of PWM writes.

Thermal policy:

- Fan starts at 42 °C, stops below 38 °C, and ramps proportionally from 25 % to
  100 % between 38 °C and 70 °C.
- Coolant pump starts at 48 °C and stops below 44 °C.
- Above 75 °C for one second, the controller **latches a fault**, drives fan and
  pump to 100 %, and sounds the alarm. It stays latched until the ACK button is
  pressed, so a transient cool-down cannot silently clear a real fault.
- `analogReadResolution(12)` and `ADC_11db` attenuation are set explicitly;
  the NTC conversion uses the Wokwi beta formula with the ESP32's 4095 full
  scale rather than the Arduino Uno's 1023.

## Run it in the browser (recommended, no toolchain)

1. Open <https://wokwi.com/projects/new/esp32>.
2. Open the **sketch.ino** tab and paste the contents of `sketch.ino`.
3. Open the **diagram.json** tab and paste the contents of `diagram.json`.
4. Press the green **Play** button and open the **Serial Monitor** (set it to **115200** baud).
5. Click the NTC part and set its `temperature` to `50`, then `80`.

Expected serial output (values will vary slightly):

```text
ms,temp_c,fan_pct,pump_pct,fault
500,25.0,0,0,0
2500,50.0,83,100,0
6000,80.0,100,100,0
7000,80.0,100,100,1
```

At 80 °C the red and green LEDs are both fully on, the buzzer sounds, `fault`
becomes `1`, and pressing **ACK** clears it.

## Run it locally (VS Code + Wokwi extension)

```bash
# once
brew install arduino-cli
arduino-cli core install esp32:esp32

# compile (from this folder)
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 --output-dir build .

# then, in VS Code with the Wokwi extension installed:
#   Command Palette -> "Wokwi: Start Simulator"
```

`wokwi.toml` already points at `build/sketch.ino.bin`.

## Why the earlier attempt stalled, and what changed

| Earlier blocker | Fix in this version |
| --- | --- |
| Missing/unavailable components | Only Wokwi-native parts: ESP32 DevKit, NTC module, 2 LEDs, 2 resistors, buzzer, pushbutton. No I2C LCD, no external sensor library, no cloud broker. |
| Library/board mismatch | Uses only `Arduino.h`, `math.h` and the ESP32 FreeRTOS headers that ship with the core. No third-party libraries to install. |
| "Simulation had issues" but no visibility | Every state change is printed as CSV, and fan/pump/fault each have a physical indicator in the diagram. |
| Nothing to show for progress | Four real tasks, a queue, a mutex, hysteresis control, a fault latch and a documented test procedure. |

## What to say about it (accurate resume wording)

> Built a FreeRTOS-based battery thermal-management controller simulation on
> ESP32 (Wokwi): four concurrent tasks for acquisition, control, telemetry and
> fault handling; queue- and mutex-protected shared state; hysteresis and
> proportional fan control; over-temperature fault latch with acknowledge.

Do not claim a real vehicle or hardware deployment. This is a simulation, and
that is fine — the value is the RTOS architecture and the control logic.

## Next steps if you want to extend it

- Replace the NTC module with a CAN-fed coolant-temperature signal (ESP32 +
  MCP2515) to mirror a real vehicle network.
- Add a second NTC for cell temperature and take the max before the control law.
- Log to an SD card instead of serial for a longer test trace.
- Add a `fault` event group and a watchdog task that resets outputs if the
  control task misses its deadline.
