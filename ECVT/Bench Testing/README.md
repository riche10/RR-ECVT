# ECVT — Bench Testing

Purpose: preserve Omari Rich's step-by-step testing version of the ECVT TCU rewrite. Automatic control is present, but the firmware starts disarmed so sensors and individual control features can be checked before enabling the complete loop.

The supplied source is unchanged. Firmware banner: `ECVT TCU Refactor Test`, version `0.2.0`.

At startup, setup initializes neutral PWM. The firmware reads sensors and calculates control values, but the output gate continues to command neutral until `a` is received through USB serial. Each reboot starts disarmed. There is no GUI dependency.

| Command | Function |
|---|---|
| `h` | Help |
| `s` | One-shot status |
| `t` | Toggle periodic serial telemetry |
| `p` | Print configuration |
| `m` | Print latest stored MoTeC data |
| `c` | Request one TCU CAN telemetry frame |
| `f` / `r` | 100 ms forward/reverse pulse while disarmed |
| `a` | Arm automatic control |
| `d` / `n` | Disarm and command neutral |

Manual pulses use the most recent stored position, and application-level command/sensor processing pauses during the 100 ms pulse. A neutral command is therefore not processed during that delay. CAN reports calculated PWM even when the arming gate holds the output neutral. These behaviors are preserved from the supplied test version.

Build in this folder with `pio run -e teensy40`. For an intentional upload, use `pio run -e teensy40 --target upload`. Compiled files are `Compiled/Bench-Testing.hex` and `Compiled/Bench-Testing.elf`.

Build verification and simulated behavior tests passed on October 2, 2026. Hardware was not uploaded to or tested. See the [project handoff](../README.md) for dependencies, CAN/pins, build versions, remaining limitations, and development history.
