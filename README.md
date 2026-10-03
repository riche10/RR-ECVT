# Rebel Racing ECVT

This repository preserves my UNLV Rebel Racing Teensy 4.0 TCU rewrite, its bench-testing and automatic versions, compiled verification artifacts, and technical handoff.

## Status

Both versions compile and pass software checks. That does **not** verify physical operation or make the controller ready to install without controlled hardware testing. I did not finish the GUI.

- **Bench Testing** retains the manual and arming controls I used to step through the control features.
- **Automatic** removes those bench controls. It can command movement after setup and has no serial stop command.

The next step is to verify neutral, direction, travel limits, RPM scaling, and CAN interpretation on the actual hardware. Known pressure, sensor, and CAN limitations are documented in the handoff.

## Start here

- [Technical handoff and build instructions](ECVT/README.md)
- [Bench Testing firmware](ECVT/Bench%20Testing)
- [Automatic firmware](ECVT/Automatic)
- [Original snapshot, build logs and tests](ECVT/Development)
- [Packaged Drive handoff](https://drive.google.com/drive/folders/17MOu5zUdLnmCDNxxZ6Qa_KUTOuj7y_u6)
- [Master archive README](https://docs.google.com/document/d/1wWvU6Ll4SLVT-M2jH4OgtgGK3EwZ3h_DStmwN0HF24g/edit)

```text
git clone https://github.com/riche10/RR-ECVT.git
```

Work on a copy or fork and record subsequent changes and test results. The original source snapshot and bundled dependency licenses remain intact. This repository starts with the supplied handoff snapshot, not reconstructed development commits. No additional project-wide license has been added.

My other Baja projects: [RR LoRa](https://github.com/riche10/RR-LoRa) · [Shock Dyno](https://github.com/riche10/RR-Shock-Dyno).
