# ECVT — Automatic

Purpose: run Omari Rich's automatic ECVT TCU controller independently after startup, with the interactive bench-testing controls removed.

Firmware banner: `ECVT TCU Refactor Automatic`, version `0.2.0`.

Setup initializes neutral PWM, then the main loop continuously reads sensors, receives CAN data, calculates control, applies the existing limits, and writes the resulting motor command. Startup does not wait for a serial connection. No GUI or arming command is required.

This version has no serial command parser, arming/disarming state, manual motor pulses, interactive telemetry toggle, one-shot status/configuration commands, or forced CAN send. **Commands such as `a`, `d`, `n`, `f`, and `r` have no effect.** USB serial provides periodic diagnostic output only, when connected. CAN telemetry continues every 250 ms.

The rewrite's sensor processing, RPM calculations, targets, calibration constants, CAN definitions, and position/PWM checks are retained. Startup neutral is initialization, not a persistent lockout: control can request movement on the first loop, including at zero primary RPM if the position allows it.

Build in this folder with `pio run -e teensy40`. For an intentional upload, use `pio run -e teensy40 --target upload`. Compiled files are `Compiled/Automatic.hex` and `Compiled/Automatic.elf`.

Build verification and simulated behavior tests passed on October 2, 2026. Hardware was not uploaded to or tested. Pressure scaling, primary RPM edge counting, actuator direction, and physical calibration still require verification. Use [Bench Testing](../Bench%20Testing/README.md) for interactive checks and see the [project handoff](../README.md) for the full development record and limitations.
