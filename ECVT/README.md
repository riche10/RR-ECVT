# Omari Rich — ECVT / TCU

This package preserves Omari Rich's ECVT TCU rewrite for UNLV Rebel Racing in two versions: **Bench Testing** and **Automatic**. Both contain the same sensor processing, RPM control calculations, CAN definitions, calibration constants, and position/PWM checks.

| Version | Purpose | Startup behavior | Serial interface |
|---|---|---|---|
| [Bench Testing](Bench%20Testing/README.md) | Check sensors and control features step by step | Neutral until explicitly armed | Status, manual pulses, arm/disarm, telemetry controls |
| [Automatic](Automatic/README.md) | Run the automatic controller after startup | Neutral during setup, then calculated motor output on each loop | Periodic diagnostic output only; no commands |

**Status, October 2, 2026:** both versions compile for Teensy 4.0. Simulated behavior tests pass. No hardware upload, physical bench test, or vehicle test was performed during this packaging work. Existing limitations below remain part of the handoff.

## Development history and preservation

Omari reports that he first had an automatically operating version, then added manual control/arming so he could test individual features step by step. Both workflows are preserved here as part of that development work.

The supplied `RR ECVT CODE 2.0` identifies its firmware as version `0.2.0`. Its source, configuration, CAN header, and bundled library are preserved byte-for-byte in **Bench Testing**. The complete supplied folder, including its existing CLion/CMake records, is preserved in `Development/Original RR ECVT CODE 2.0.zip`. The desktop source folder was not edited.

**Automatic** retains the rewrite's core controller and removes the bench interaction layer: the arming gate, manual pulses, serial command parser/help, command-driven status/configuration readouts, forced-send control, cached terminal status, and the wait for a serial connection. It prints periodic diagnostics when USB serial is connected. Its firmware name distinguishes it from Bench Testing; the base firmware version remains `0.2.0`.

The original pre-rewrite V5 ZIP and Mason D's separately attributed source/hardware documentation are reference material identified during inspection. They are not merged into these two TCU variants. This package does not modify PDM or DCM firmware. No original `.git` history was present in the supplied rewrite folder; the source snapshot, hashes, and source comparison record its provenance without inventing commits.

## What was implemented

The rewrite organizes setup, sensing, filtering, control, CAN, and output into functions and named data structures. It retains the two RPM targets and implements explicit rear-wheel input configuration, RPM scaling from configured magnet/tooth counts, bounded position percentage, complete RPM state reset on timeout, and directional checks at the position limits. Bench Testing adds the interactive tools used to test those features.

The GUI was not completed, according to Omari. Neither version requires a GUI: Bench Testing uses a serial terminal, and Automatic runs independently.

## Hardware and software

The firmware targets a Teensy 4.0 with the Arduino framework and the included FlexCAN_T4 library. The TCU reads two RPM sensors and a sheave-position potentiometer, calculates a motor command, and sends telemetry on CAN2. MoTeC data is received through the shared CAN definitions.

| Signal | Teensy pin / configuration |
|---|---|
| Primary RPM | 22, CHANGE interrupt, configured count 10/revolution |
| Rear-wheel RPM | 15, RISING interrupt, configured count 44/revolution |
| Sheave potentiometer | 16, 12-bit ADC, 32-sample ADC averaging |
| Motor PWM | 4, 100 Hz, 12-bit output, neutral 615 |
| CAN | CAN2, 1 Mbit/s, standard 11-bit IDs |
| Serial diagnostics | 115200 baud setting |

Control constants: 20-sample RPM averaging; 2400 RPM torque target; 3000 RPM power target above 77 rear-wheel RPM; 50 RPM deadband; 400 RPM saturation error. Position calibration remains 1175–3660 ADC counts with control limits 1468–3294. Confirm these against the actual hardware.

CAN ID `0x130` is transmitted every 250 ms. Bytes 0–1 contain constrained sheave position, 2–3 primary average RPM, 4–5 rear-wheel average RPM, and 6–7 calculated PWM, each as a big-endian 16-bit value. MoTeC ID `0x100` is decoded as brake toggle, brake pressure, throttle position, and CVT pressure in bytes 0–3; its physical units and scaling remain unverified.

## Build and upload

Each version is a complete, independent PlatformIO project with `src`, `include`, `lib`, and `platformio.ini`. Open the chosen version's folder as the project root. Keep its vendored library with it; no separate copy of Mason's shared CAN library is needed.

From the selected project folder:

```text
pio run -e teensy40
```

This creates `.pio/build/teensy40/firmware.hex` and `firmware.elf`. The versions built for this package are also kept in each folder's `Compiled` directory with distinct filenames.

For an intentional hardware upload from the selected project folder:

```text
pio run -e teensy40 --target upload
```

The supplied configuration uses PlatformIO's default Teensy GUI uploader. Use the Teensy program button if the uploader requests it. A serial monitor can be opened with `pio device monitor --baud 115200`; select the correct device port if required. These are instructions, not actions performed as part of this package. See the official [PlatformIO Teensy 4.0 documentation](https://docs.platformio.org/en/latest/boards/teensy/teensy40.html).

Automatic begins controlling after setup and has **no serial neutral/stop command**. Choose Bench Testing for interactive bring-up. The Automatic control calculation can request movement even with zero primary RPM if position allows it; no engine-running gate was added.

## Verification and compiled artifacts

Verified environment:

| Component | Version |
|---|---|
| PlatformIO Core | 6.2.0 |
| Teensy platform | 6.0.0 |
| Arduino Teensy framework | 1.162.0 / Teensyduino 1.62 |
| ARM compiler package | 1.150201.0 / GCC 15.2.1 |
| Teensy tools | 1.162.0 |
| SCons package | 4.41101.0 |
| FlexCAN_T4 | Supplied vendored snapshot; file hashes recorded |

The original `platformio.ini` remains unpinned. Future package resolution may differ; use the recorded versions when reproducing this build. Full build logs are in `Development`. Compiler warnings originate in the supplied FlexCAN library and are retained in those logs; the library was not changed.

The tests in `Development/tests` compile the real source with mocked Arduino/CAN interfaces. They verify startup without a serial connection, neutral setup, variant-specific command behavior, CAN payloads, deadband and position/PWM limits. They also compare 120 control-input cases across both variants and confirm 21 shared functions remain identical. These tests do not validate electrical behavior, interrupt timing, CAN delivery, or mechanics. Results are in `Development/verification-results.json`.

The `Development/variant-diff.patch` records the source difference. `Development/SHA256SUMS.json` identifies package files and compiled outputs. The `.elf` files retain compiled/debug information; `.hex` files are the firmware images. All compiled outputs in this package were generated during this verification, not recovered from a past deployment.

## Known limitations and next step

- Pressure cutoff remains ineffective: a one-byte value cannot reach the configured threshold of 450. Confirm the MoTeC format/scaling before changing it.
- Primary RPM uses CHANGE interrupts with a conversion based on 10 events per revolution. Verify the actual edge count/waveform against a known RPM input.
- CAN values have no freshness timeout; received length is not validated. The send helper reports success without checking the driver's return value.
- Potentiometer clamping hides out-of-range readings instead of reporting a sensor fault. Direction and limits depend on physical calibration.
- In Bench Testing, manual pulses block application-level updates for 100 ms and can use stale position data. Its CAN PWM field reports the calculated value even while the arming gate commands neutral. Automatic has neither the manual-pulse path nor the arming gate.
- The vendor library produced warnings with the recorded compiler. A successful build does not resolve those warnings or establish hardware operation.

The next effective engineering step is controlled hardware verification using Bench Testing: confirm neutral, sensor values/RPM scaling, direction, travel limits, and CAN interpretation. Then evaluate Automatic against the verified hardware configuration. Preserve the current snapshots and record any subsequent fixes or tests separately.

## Archive location

My permanent source repository is [riche10/RR-ECVT](https://github.com/riche10/RR-ECVT). Clone it with `git clone https://github.com/riche10/RR-ECVT.git` or fork it on GitHub.

The [ECVT Drive folder](https://drive.google.com/drive/folders/17MOu5zUdLnmCDNxxZ6Qa_KUTOuj7y_u6) contains the packaged handoff inside [OMR Archive Code Folders](https://drive.google.com/drive/folders/1YMSVlIcbM706c-cv89uMgr0dELSpGD02). The master README in OMR links all three projects. GitHub history starts with the supplied archive snapshot and records publication updates; it does not recreate missing development commits.
