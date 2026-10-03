#include <Arduino.h>
#include <MyCANBUS.h>

/*
    ECVT TCU Refactored Test Firmware
    ---------------------------------
    This version keeps the original TCU control logic but organizes it into
    readable functions and adds USB Serial terminal commands for bench testing.

    IMPORTANT:
    - Motor output is DISARMED by default.
    - When disarmed, the motor PWM output is forced to neutral.
    - Manual forward/reverse commands use short 100 ms pulses.
    - Automatic mode should not be armed until sensor values, actuator
      direction, and safety limits have been verified.

    USB Serial Terminal Commands
    ----------------------------
    h = show help
    s = print one-shot status
    t = toggle periodic telemetry
    f = manual 100 ms forward pulse
    r = manual 100 ms reverse pulse
    a = arm automatic RPM control
    d = disarm automatic control
    n = force neutral and disarm
    c = force-send one TCU CAN telemetry frame
    p = print TCU configuration
    m = print latest MoTeC data
*/

/////////////////////////// Firmware Information ///////////////////////////

constexpr char FIRMWARE_NAME[] = "ECVT TCU Refactor Test";
constexpr char FIRMWARE_VERSION[] = "0.2.0";

/////////////////////////// Pin Definitions ///////////////////////////

namespace Pins {
    constexpr uint8_t PRIMARY_RPM = 22;
    constexpr uint8_t REAR_WHEEL_RPM = 15;
    constexpr uint8_t LINEAR_POT = 16;
    constexpr uint8_t MOTOR_PWM = 4;
}

/////////////////////////// TCU Configuration ///////////////////////////

struct TcuConfig {
    // Serial and timing
    uint32_t serialBaud = 115200;
    uint32_t canTransmitPeriodMs = 250;
    uint32_t serialPrintPeriodMs = 250;
    uint32_t manualPulseDurationMs = 100;

    // Physical sensor parameters
    uint8_t primaryMagnetsPerRev = 10;
    uint8_t rearWheelTeeth = 44;

    // Potentiometer / actuator position limits
    uint16_t trueSheaveMin = 1175;
    uint16_t trueSheaveMax = 3660;
    uint16_t sheaveMin = 1468;
    uint16_t sheaveMax = 3294;

    // RPM control parameters
    uint16_t rpmDeadband = 50;
    uint32_t saturationError = 400;
    uint16_t targetRpmTorque = 2400;
    uint16_t targetRpmPower = 3000;
    uint16_t powerTorqueSwitchPoint = 77;

    // Motor controller PWM parameters
    uint16_t pwmNeutral = 615;

    // Current assumed forward/downshift direction.
    uint16_t pwmForwardStart = 717;
    uint16_t pwmForwardMin = 625;
    uint16_t pwmForwardMax = 819;

    // Current assumed reverse/upshift direction.
    uint16_t pwmReverseStart = 513;
    uint16_t pwmReverseMin = 604;
    uint16_t pwmReverseMax = 410;

    // Sensor timeout thresholds
    uint32_t primaryTimeoutUs = 8000;
    uint32_t rearWheelTimeoutUs = 227272;

    // TODO:
    // Current MoTeC decoding stores CVT pressure in one byte.
    // A one-byte value cannot reach 450, so this limit is not currently useful
    // until the actual pressure message format and scaling are confirmed.
    uint16_t cvtPressureLimit = 450;
};

constexpr uint8_t MOVING_AVG_SIZE = 20;

TcuConfig config;

/////////////////////////// Runtime Data Structures ///////////////////////////

struct RpmState {
    uint16_t history[MOVING_AVG_SIZE] = {0};
    uint8_t index = 0;
    uint16_t rawRpm = 0;
    uint16_t avgRpm = 0;
};

struct SensorData {
    uint16_t rawPotValue = 0;
    float potPercent = 0.0f;

    uint16_t primaryRawRpm = 0;
    uint16_t primaryAvgRpm = 0;

    uint16_t rearWheelRawRpm = 0;
    uint16_t rearWheelAvgRpm = 0;
};

struct ControlData {
    uint16_t activeTargetRpm = 0;
    int32_t rpmError = 0;
    uint16_t calculatedPwm = 0;
};

struct MotecData {
    bool brakeToggle = false;
    uint8_t brakePressure = 0;
    uint8_t throttlePosition = 0;

    // TODO: Verify whether CVT pressure is actually one or two CAN bytes.
    uint8_t cvtPressure = 0;
};

struct TerminalState {
    // Automatic control is intentionally disabled on every boot.
    bool automaticControlArmed = false;

    // Periodic serial output is disabled so command typing is readable.
    bool serialTelemetryEnabled = false;

    // Used to prevent overlapping manual test commands.
    bool manualPulseActive = false;
};

RpmState primaryRpm;
RpmState rearWheelRpm;

MotecData motec;
TerminalState terminal;

SensorData lastSensors;
ControlData lastControl;

/////////////////////////// ISR Variables ///////////////////////////

volatile uint32_t v_PR_Last_Pulse = 0;
volatile uint32_t v_PR_Interval_us = 0;
volatile bool v_PR_Pulse_Available = false;

volatile uint32_t v_RW_Last_Pulse = 0;
volatile uint32_t v_RW_Interval_us = 0;
volatile bool v_RW_Pulse_Available = false;

/////////////////////////// Function Declarations ///////////////////////////

void PR_RPM_ISR();
void RW_RPM_ISR();

void setupSerial();
void setupCan();
void setupInputs();
void setupMotorOutput();
void setupInterrupts();

SensorData readSensors();
uint16_t readSheavePosition();
float calculatePotPercent(uint16_t rawPotValue);

bool capturePrimaryInterval(uint32_t &intervalUs);
bool captureRearWheelInterval(uint32_t &intervalUs);

void updateRpmFromInterval(
    RpmState &rpmState,
    uint32_t intervalUs,
    uint32_t rpmScale
);

uint16_t combineAverage(
    const uint16_t history[],
    uint8_t size
);

void applyRpmTimeouts();
void clearRpmState(RpmState &rpmState);

ControlData calculateControl(const SensorData &sensors);
uint16_t chooseActiveTarget(uint16_t rearWheelAvgRpm);
uint16_t calculateMotorPwm(int32_t error, uint16_t rawPotValue);
uint16_t applySafetyClamp(uint16_t pwmOutput, uint16_t rawPotValue);

void receiveCanData();

void sendTcuTelemetry(
    const SensorData &sensors,
    uint16_t pwmOutput,
    bool forceSend = false
);

void writeAutomaticMotorCommand(uint16_t pwmOutput);
void forceNeutral();

void pulseMotor(
    uint16_t pwmValue,
    const char *directionName
);

void printDebugData(
    const SensorData &sensors,
    const ControlData &control
);

void handleSerialCommands();
void printTerminalHelp();
void printOneShotStatus();
void printConfigSummary();
void printMotecData();

/////////////////////////// Setup ///////////////////////////

void setup() {
    setupSerial();
    setupCan();
    setupInputs();
    setupMotorOutput();
    setupInterrupts();

    if (Serial) {
        Serial.println();
        Serial.println("========================================");
        Serial.println(FIRMWARE_NAME);
        Serial.print("Firmware version: ");
        Serial.println(FIRMWARE_VERSION);
        Serial.println("========================================");

        Serial.println("TCU initialized.");
        Serial.println("Automatic motor control is DISARMED.");
        Serial.println("Motor output is being held at neutral.");

        printTerminalHelp();
    }
}

void setupSerial() {
    Serial.begin(config.serialBaud);

    while (!Serial && millis() < 2000) {
        // Wait briefly for USB Serial, then continue standalone.
    }
}

void setupCan() {
    /*
        false means this board does not use the optional CAN transceiver
        shutdown/standby pins handled by MyCanStart().
    */
    MyCanStart(false);
}

void setupInputs() {
    pinMode(Pins::PRIMARY_RPM, INPUT);
    pinMode(Pins::REAR_WHEEL_RPM, INPUT);
    pinMode(Pins::LINEAR_POT, INPUT);

    analogReadResolution(12);
    analogReadAveraging(32);
}

void setupMotorOutput() {
    pinMode(Pins::MOTOR_PWM, OUTPUT);

    analogWriteFrequency(Pins::MOTOR_PWM, 100);
    analogWriteResolution(12);

    forceNeutral();
}

void setupInterrupts() {
    /*
        TODO:
        Confirm whether primary RPM should use CHANGE or RISING.
        CHANGE may double-count pulses depending on the sensor waveform.
    */
    attachInterrupt(
        digitalPinToInterrupt(Pins::PRIMARY_RPM),
        PR_RPM_ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(Pins::REAR_WHEEL_RPM),
        RW_RPM_ISR,
        RISING
    );
}

/////////////////////////// Main Loop ///////////////////////////

void loop() {
    handleSerialCommands();

    SensorData sensors = readSensors();

    receiveCanData();

    ControlData control = calculateControl(sensors);

    control.calculatedPwm = applySafetyClamp(
        control.calculatedPwm,
        sensors.rawPotValue
    );

    lastSensors = sensors;
    lastControl = control;

    sendTcuTelemetry(
        sensors,
        control.calculatedPwm
    );

    writeAutomaticMotorCommand(
        control.calculatedPwm
    );

    printDebugData(
        sensors,
        control
    );
}

/////////////////////////// Sensor Reading ///////////////////////////

SensorData readSensors() {
    SensorData sensors;

    sensors.rawPotValue = readSheavePosition();

    sensors.potPercent = calculatePotPercent(
        sensors.rawPotValue
    );

    uint32_t intervalUs = 0;

    if (
        capturePrimaryInterval(intervalUs) &&
        intervalUs > 0
    ) {
        const uint32_t primaryRpmScale =
            60000000UL / config.primaryMagnetsPerRev;

        updateRpmFromInterval(
            primaryRpm,
            intervalUs,
            primaryRpmScale
        );
    }

    if (
        captureRearWheelInterval(intervalUs) &&
        intervalUs > 0
    ) {
        const uint32_t rearWheelRpmScale =
            60000000UL / config.rearWheelTeeth;

        updateRpmFromInterval(
            rearWheelRpm,
            intervalUs,
            rearWheelRpmScale
        );
    }

    applyRpmTimeouts();

    sensors.primaryRawRpm = primaryRpm.rawRpm;
    sensors.primaryAvgRpm = primaryRpm.avgRpm;

    sensors.rearWheelRawRpm = rearWheelRpm.rawRpm;
    sensors.rearWheelAvgRpm = rearWheelRpm.avgRpm;

    return sensors;
}

uint16_t readSheavePosition() {
    const uint16_t rawPotValue =
        analogRead(Pins::LINEAR_POT);

    /*
        This preserves the original behavior.

        Note:
        Constraining the value hides readings outside the physical range.
        A future version should separately report an out-of-range sensor fault.
    */
    return constrain(
        rawPotValue,
        config.trueSheaveMin,
        config.trueSheaveMax
    );
}

float calculatePotPercent(uint16_t rawPotValue) {
    if (rawPotValue <= config.sheaveMin) {
        return 0.0f;
    }

    if (rawPotValue >= config.sheaveMax) {
        return 100.0f;
    }

    const float numerator =
        static_cast<float>(
            rawPotValue - config.sheaveMin
        );

    const float denominator =
        static_cast<float>(
            config.sheaveMax - config.sheaveMin
        );

    return (numerator / denominator) * 100.0f;
}

/////////////////////////// RPM Capture / Filtering ///////////////////////////

bool capturePrimaryInterval(uint32_t &intervalUs) {
    if (!v_PR_Pulse_Available) {
        return false;
    }

    noInterrupts();

    intervalUs = v_PR_Interval_us;
    v_PR_Pulse_Available = false;

    interrupts();

    return true;
}

bool captureRearWheelInterval(uint32_t &intervalUs) {
    if (!v_RW_Pulse_Available) {
        return false;
    }

    noInterrupts();

    intervalUs = v_RW_Interval_us;
    v_RW_Pulse_Available = false;

    interrupts();

    return true;
}

void updateRpmFromInterval(
    RpmState &rpmState,
    uint32_t intervalUs,
    uint32_t rpmScale
) {
    if (intervalUs == 0) {
        return;
    }

    rpmState.rawRpm =
        static_cast<uint16_t>(
            rpmScale / intervalUs
        );

    rpmState.history[rpmState.index] =
        rpmState.rawRpm;

    rpmState.index =
        (rpmState.index + 1) % MOVING_AVG_SIZE;

    rpmState.avgRpm = combineAverage(
        rpmState.history,
        MOVING_AVG_SIZE
    );
}

uint16_t combineAverage(
    const uint16_t history[],
    uint8_t size
) {
    if (size == 0) {
        return 0;
    }

    uint32_t total = 0;

    for (uint8_t i = 0; i < size; i++) {
        total += history[i];
    }

    return static_cast<uint16_t>(
        total / size
    );
}

void applyRpmTimeouts() {
    const uint32_t now = micros();

    if (
        now - v_PR_Last_Pulse >
        config.primaryTimeoutUs
    ) {
        if (
            primaryRpm.rawRpm != 0 ||
            primaryRpm.avgRpm != 0
        ) {
            clearRpmState(primaryRpm);
        }
    }

    if (
        now - v_RW_Last_Pulse >
        config.rearWheelTimeoutUs
    ) {
        if (
            rearWheelRpm.rawRpm != 0 ||
            rearWheelRpm.avgRpm != 0
        ) {
            clearRpmState(rearWheelRpm);
        }
    }
}

void clearRpmState(RpmState &rpmState) {
    rpmState.rawRpm = 0;
    rpmState.avgRpm = 0;
    rpmState.index = 0;

    for (
        uint8_t i = 0;
        i < MOVING_AVG_SIZE;
        i++
    ) {
        rpmState.history[i] = 0;
    }
}

/////////////////////////// Control Logic ///////////////////////////

ControlData calculateControl(
    const SensorData &sensors
) {
    ControlData control;

    control.activeTargetRpm =
        chooseActiveTarget(
            sensors.rearWheelAvgRpm
        );

    control.rpmError =
        static_cast<int32_t>(
            sensors.primaryAvgRpm
        ) -
        static_cast<int32_t>(
            control.activeTargetRpm
        );

    control.calculatedPwm =
        calculateMotorPwm(
            control.rpmError,
            sensors.rawPotValue
        );

    return control;
}

uint16_t chooseActiveTarget(
    uint16_t rearWheelAvgRpm
) {
    if (
        rearWheelAvgRpm >
        config.powerTorqueSwitchPoint
    ) {
        return config.targetRpmPower;
    }

    return config.targetRpmTorque;
}

uint16_t calculateMotorPwm(
    int32_t error,
    uint16_t rawPotValue
) {
    const uint32_t absoluteError =
        error < 0
            ? static_cast<uint32_t>(-error)
            : static_cast<uint32_t>(error);

    if (
        absoluteError <=
        config.rpmDeadband
    ) {
        return config.pwmNeutral;
    }

    if (error > 0) {
        /*
            Primary RPM is too high.

            Current original-code assumption:
            reverse PWM causes an upshift.

            This direction still needs physical verification.
        */
        if (
            rawPotValue >=
            config.sheaveMax
        ) {
            return config.pwmNeutral;
        }

        long mappedPwm = map(
            error,
            config.rpmDeadband,
            config.saturationError,
            config.pwmReverseStart,
            config.pwmReverseMax
        );

        mappedPwm = constrain(
            mappedPwm,
            static_cast<long>(
                config.pwmReverseMax
            ),
            static_cast<long>(
                config.pwmReverseStart
            )
        );

        return static_cast<uint16_t>(
            mappedPwm
        );
    }

    /*
        Primary RPM is too low.

        Current original-code assumption:
        forward PWM causes a downshift.

        This direction still needs physical verification.
    */
    if (
        rawPotValue <=
        config.sheaveMin
    ) {
        return config.pwmNeutral;
    }

    long mappedPwm = map(
        absoluteError,
        config.rpmDeadband,
        config.saturationError,
        config.pwmForwardStart,
        config.pwmForwardMax
    );

    mappedPwm = constrain(
        mappedPwm,
        static_cast<long>(
            config.pwmForwardStart
        ),
        static_cast<long>(
            config.pwmForwardMax
        )
    );

    return static_cast<uint16_t>(
        mappedPwm
    );
}

uint16_t applySafetyClamp(
    uint16_t pwmOutput,
    uint16_t rawPotValue
) {
    if (pwmOutput > 4095) {
        return config.pwmNeutral;
    }

    /*
        TODO:
        The current CVT pressure value is one byte and therefore cannot
        reach the configured value of 450.

        This remains disabled in practical terms until the CAN format is
        verified.
    */
    if (
        motec.cvtPressure >=
        config.cvtPressureLimit
    ) {
        return config.pwmNeutral;
    }

    /*
        General hard-limit checks.

        These prevent an automatic command from continuing farther beyond
        the configured position range.
    */
    if (
        rawPotValue <= config.sheaveMin &&
        pwmOutput > config.pwmNeutral
    ) {
        return config.pwmNeutral;
    }

    if (
        rawPotValue >= config.sheaveMax &&
        pwmOutput < config.pwmNeutral
    ) {
        return config.pwmNeutral;
    }

    return pwmOutput;
}

/////////////////////////// CAN Communication ///////////////////////////

void receiveCanData() {
    Can2.events();

    motec.brakeToggle =
        MOTEC_Data_RX0.x[0] != 0;

    motec.brakePressure =
        MOTEC_Data_RX0.x[1];

    motec.throttlePosition =
        MOTEC_Data_RX0.x[2];

    motec.cvtPressure =
        MOTEC_Data_RX0.x[3];
}

void sendTcuTelemetry(
    const SensorData &sensors,
    uint16_t pwmOutput,
    bool forceSend
) {
    static uint32_t lastCanSendMs = 0;

    const uint32_t now = millis();

    if (
        !forceSend &&
        now - lastCanSendMs <
        config.canTransmitPeriodMs
    ) {
        return;
    }

    lastCanSendMs = now;

    TCU_Data_TX.id = TCU_id0;

    TCU_Data_TX.x[0] =
        static_cast<uint8_t>(
            sensors.rawPotValue >> 8
        );

    TCU_Data_TX.x[1] =
        static_cast<uint8_t>(
            sensors.rawPotValue & 0xFF
        );

    TCU_Data_TX.x[2] =
        static_cast<uint8_t>(
            sensors.primaryAvgRpm >> 8
        );

    TCU_Data_TX.x[3] =
        static_cast<uint8_t>(
            sensors.primaryAvgRpm & 0xFF
        );

    TCU_Data_TX.x[4] =
        static_cast<uint8_t>(
            sensors.rearWheelAvgRpm >> 8
        );

    TCU_Data_TX.x[5] =
        static_cast<uint8_t>(
            sensors.rearWheelAvgRpm & 0xFF
        );

    TCU_Data_TX.x[6] =
        static_cast<uint8_t>(
            pwmOutput >> 8
        );

    TCU_Data_TX.x[7] =
        static_cast<uint8_t>(
            pwmOutput & 0xFF
        );

    MyCanSend(TCU_Data_TX);

    if (Serial && forceSend) {
        Serial.println(
            "Forced TCU CAN telemetry sent on ID 0x130."
        );
    }
}

/////////////////////////// Motor Output ///////////////////////////

void forceNeutral() {
    analogWrite(
        Pins::MOTOR_PWM,
        config.pwmNeutral
    );
}

void writeAutomaticMotorCommand(
    uint16_t pwmOutput
) {
    if (terminal.manualPulseActive) {
        return;
    }

    if (!terminal.automaticControlArmed) {
        forceNeutral();
        return;
    }

    analogWrite(
        Pins::MOTOR_PWM,
        pwmOutput
    );
}

void pulseMotor(
    uint16_t pwmValue,
    const char *directionName
) {
    if (terminal.automaticControlArmed) {
        Serial.println(
            "Manual pulse blocked: automatic control is ARMED."
        );

        Serial.println(
            "Type 'd' or 'n' before manual jogging."
        );

        return;
    }

    if (terminal.manualPulseActive) {
        Serial.println(
            "Manual pulse already active."
        );

        return;
    }

    /*
        Prevent the manual command from moving farther into a configured
        position limit.

        This protection depends on the assumed direction mapping and should
        be verified during testing.
    */
    if (
        pwmValue > config.pwmNeutral &&
        lastSensors.rawPotValue <=
            config.sheaveMin
    ) {
        Serial.println(
            "Forward pulse blocked by configured minimum position."
        );

        return;
    }

    if (
        pwmValue < config.pwmNeutral &&
        lastSensors.rawPotValue >=
            config.sheaveMax
    ) {
        Serial.println(
            "Reverse pulse blocked by configured maximum position."
        );

        return;
    }

    terminal.manualPulseActive = true;

    Serial.println();
    Serial.println("WARNING: Manual motor pulse starting.");

    Serial.print("Direction label: ");
    Serial.println(directionName);

    Serial.print("PWM command: ");
    Serial.println(pwmValue);

    Serial.print("Pulse duration: ");
    Serial.print(config.manualPulseDurationMs);
    Serial.println(" ms");

    Serial.print("Position before pulse: ");
    Serial.println(lastSensors.rawPotValue);

    analogWrite(
        Pins::MOTOR_PWM,
        pwmValue
    );

    delay(config.manualPulseDurationMs);

    forceNeutral();

    terminal.manualPulseActive = false;

    Serial.println(
        "Manual pulse completed. PWM returned to neutral."
    );

    Serial.println();
}

/////////////////////////// Debug Printing ///////////////////////////

void printDebugData(
    const SensorData &sensors,
    const ControlData &control
) {
    if (
        !Serial ||
        !terminal.serialTelemetryEnabled
    ) {
        return;
    }

    static uint32_t lastPrintMs = 0;

    const uint32_t now = millis();

    if (
        now - lastPrintMs <
        config.serialPrintPeriodMs
    ) {
        return;
    }

    lastPrintMs = now;

    const uint16_t secondaryRpm =
        static_cast<uint16_t>(
            (44.0f / 14.0f) *
            sensors.rearWheelAvgRpm
        );

    float ratio = 0.0f;

    if (secondaryRpm != 0) {
        ratio =
            static_cast<float>(
                sensors.primaryAvgRpm
            ) /
            static_cast<float>(
                secondaryRpm
            );
    }

    const float mph =
        sensors.rearWheelAvgRpm *
        0.06842f;

    Serial.println();
    Serial.println("---------- TCU Live Data ----------");

    Serial.print("Raw Position: ");
    Serial.print(sensors.rawPotValue);

    Serial.print(" | Position: ");
    Serial.print(sensors.potPercent, 2);
    Serial.println("%");

    Serial.print("Primary RPM Raw: ");
    Serial.print(sensors.primaryRawRpm);

    Serial.print(" | Average: ");
    Serial.println(sensors.primaryAvgRpm);

    Serial.print("Rear Wheel RPM Raw: ");
    Serial.print(sensors.rearWheelRawRpm);

    Serial.print(" | Average: ");
    Serial.println(sensors.rearWheelAvgRpm);

    Serial.print("Estimated Secondary RPM: ");
    Serial.println(secondaryRpm);

    Serial.print("Estimated Ratio: ");
    Serial.println(ratio, 2);

    Serial.print("Estimated MPH: ");
    Serial.println(mph, 2);

    Serial.print("Target RPM: ");
    Serial.println(control.activeTargetRpm);

    Serial.print("RPM Error: ");
    Serial.println(control.rpmError);

    Serial.print("Calculated PWM: ");
    Serial.println(control.calculatedPwm);

    Serial.print("Actual Automatic Output: ");

    if (terminal.automaticControlArmed) {
        Serial.println(control.calculatedPwm);
    } else {
        Serial.print(config.pwmNeutral);
        Serial.println(" (automatic control disarmed)");
    }

    Serial.println("-----------------------------------");
}

/////////////////////////// Terminal Commands ///////////////////////////

void handleSerialCommands() {
    if (!Serial) {
        return;
    }

    while (Serial.available() > 0) {
        const char command = Serial.read();

        if (
            command == '\n' ||
            command == '\r' ||
            command == ' ' ||
            command == '\t'
        ) {
            continue;
        }

        switch (command) {
            case 'h':
            case 'H':
                printTerminalHelp();
                break;

            case 's':
            case 'S':
                printOneShotStatus();
                break;

            case 't':
            case 'T':
                terminal.serialTelemetryEnabled =
                    !terminal.serialTelemetryEnabled;

                Serial.print(
                    "Periodic serial telemetry: "
                );

                Serial.println(
                    terminal.serialTelemetryEnabled
                        ? "ENABLED"
                        : "DISABLED"
                );

                break;

            case 'f':
            case 'F':
                pulseMotor(
                    config.pwmForwardStart,
                    "FORWARD"
                );
                break;

            case 'r':
            case 'R':
                pulseMotor(
                    config.pwmReverseStart,
                    "REVERSE"
                );
                break;

            case 'a':
            case 'A':
                terminal.automaticControlArmed = true;

                Serial.println();
                Serial.println(
                    "WARNING: AUTOMATIC CONTROL ARMED."
                );

                Serial.println(
                    "PWM will now follow the RPM control calculation."
                );

                Serial.println(
                    "Type 'n' immediately to return to neutral."
                );

                Serial.println();
                break;

            case 'd':
            case 'D':
                terminal.automaticControlArmed = false;

                forceNeutral();

                Serial.println(
                    "Automatic control DISARMED."
                );

                Serial.println(
                    "PWM forced to neutral."
                );

                break;

            case 'n':
            case 'N':
                terminal.automaticControlArmed = false;
                terminal.manualPulseActive = false;

                forceNeutral();

                Serial.println(
                    "EMERGENCY NEUTRAL COMMAND APPLIED."
                );

                Serial.println(
                    "Automatic control is DISARMED."
                );

                break;

            case 'c':
            case 'C':
                sendTcuTelemetry(
                    lastSensors,
                    lastControl.calculatedPwm,
                    true
                );

                break;

            case 'p':
            case 'P':
                printConfigSummary();
                break;

            case 'm':
            case 'M':
                printMotecData();
                break;

            default:
                Serial.print(
                    "Unknown command: "
                );

                Serial.println(command);

                Serial.println(
                    "Type 'h' for the command list."
                );

                break;
        }
    }
}

void printTerminalHelp() {
    Serial.println();
    Serial.println(
        "========== ECVT TCU Terminal Commands =========="
    );

    Serial.println(
        "h = show this help menu"
    );

    Serial.println(
        "s = print one-shot TCU status"
    );

    Serial.println(
        "t = toggle periodic live telemetry"
    );

    Serial.println(
        "f = manual 100 ms FORWARD PWM pulse"
    );

    Serial.println(
        "r = manual 100 ms REVERSE PWM pulse"
    );

    Serial.println(
        "a = ARM the full automatic RPM controller"
    );

    Serial.println(
        "d = DISARM automatic control"
    );

    Serial.println(
        "n = force NEUTRAL and disarm"
    );

    Serial.println(
        "c = force-send one CAN telemetry frame"
    );

    Serial.println(
        "p = print TCU configuration"
    );

    Serial.println(
        "m = print latest MoTeC CAN values"
    );

    Serial.println();

    Serial.println(
        "Safety default: automatic control is DISARMED."
    );

    Serial.println(
        "Use f/r for initial direction testing."
    );

    Serial.println(
        "Do not use 'a' for the first motor movement test."
    );

    Serial.println(
        "================================================"
    );

    Serial.println();
}

void printOneShotStatus() {
    Serial.println();
    Serial.println(
        "========== TCU One-Shot Status =========="
    );

    Serial.print(
        "Firmware: "
    );

    Serial.print(FIRMWARE_NAME);
    Serial.print(" ");
    Serial.println(FIRMWARE_VERSION);

    Serial.print(
        "Automatic control: "
    );

    Serial.println(
        terminal.automaticControlArmed
            ? "ARMED"
            : "DISARMED"
    );

    Serial.print(
        "Manual pulse active: "
    );

    Serial.println(
        terminal.manualPulseActive
            ? "YES"
            : "NO"
    );

    Serial.print(
        "Raw position: "
    );

    Serial.println(
        lastSensors.rawPotValue
    );

    Serial.print(
        "Position percent: "
    );

    Serial.print(
        lastSensors.potPercent,
        2
    );

    Serial.println("%");

    Serial.print(
        "Primary RPM raw / average: "
    );

    Serial.print(
        lastSensors.primaryRawRpm
    );

    Serial.print(" / ");

    Serial.println(
        lastSensors.primaryAvgRpm
    );

    Serial.print(
        "Rear wheel RPM raw / average: "
    );

    Serial.print(
        lastSensors.rearWheelRawRpm
    );

    Serial.print(" / ");

    Serial.println(
        lastSensors.rearWheelAvgRpm
    );

    Serial.print(
        "Active target RPM: "
    );

    Serial.println(
        lastControl.activeTargetRpm
    );

    Serial.print(
        "RPM error: "
    );

    Serial.println(
        lastControl.rpmError
    );

    Serial.print(
        "Calculated PWM: "
    );

    Serial.println(
        lastControl.calculatedPwm
    );

    Serial.print(
        "PWM currently allowed: "
    );

    Serial.println(
        terminal.automaticControlArmed
            ? lastControl.calculatedPwm
            : config.pwmNeutral
    );

    Serial.println(
        "========================================="
    );

    Serial.println();
}

void printConfigSummary() {
    Serial.println();
    Serial.println(
        "========== TCU Configuration =========="
    );

    Serial.print("Serial baud: ");
    Serial.println(config.serialBaud);

    Serial.print("CAN telemetry period: ");
    Serial.print(config.canTransmitPeriodMs);
    Serial.println(" ms");

    Serial.print("Serial telemetry period: ");
    Serial.print(config.serialPrintPeriodMs);
    Serial.println(" ms");

    Serial.print("Manual pulse duration: ");
    Serial.print(config.manualPulseDurationMs);
    Serial.println(" ms");

    Serial.print("Primary magnets per revolution: ");
    Serial.println(config.primaryMagnetsPerRev);

    Serial.print("Rear wheel teeth: ");
    Serial.println(config.rearWheelTeeth);

    Serial.print("True position min / max: ");
    Serial.print(config.trueSheaveMin);
    Serial.print(" / ");
    Serial.println(config.trueSheaveMax);

    Serial.print("Software position min / max: ");
    Serial.print(config.sheaveMin);
    Serial.print(" / ");
    Serial.println(config.sheaveMax);

    Serial.print("Torque target RPM: ");
    Serial.println(config.targetRpmTorque);

    Serial.print("Power target RPM: ");
    Serial.println(config.targetRpmPower);

    Serial.print("Target switch point: ");
    Serial.println(config.powerTorqueSwitchPoint);

    Serial.print("RPM deadband: ");
    Serial.println(config.rpmDeadband);

    Serial.print("Saturation error: ");
    Serial.println(config.saturationError);

    Serial.print("PWM neutral: ");
    Serial.println(config.pwmNeutral);

    Serial.print("PWM forward start / max: ");
    Serial.print(config.pwmForwardStart);
    Serial.print(" / ");
    Serial.println(config.pwmForwardMax);

    Serial.print("PWM reverse start / max: ");
    Serial.print(config.pwmReverseStart);
    Serial.print(" / ");
    Serial.println(config.pwmReverseMax);

    Serial.print("CVT pressure cutoff: ");
    Serial.println(config.cvtPressureLimit);

    Serial.println(
        "======================================="
    );

    Serial.println();
}

void printMotecData() {
    Serial.println();
    Serial.println(
        "========== Latest MoTeC Data =========="
    );

    Serial.print("Brake toggle: ");
    Serial.println(
        motec.brakeToggle
            ? "ON"
            : "OFF"
    );

    Serial.print("Brake pressure raw: ");
    Serial.println(motec.brakePressure);

    Serial.print("Throttle position raw: ");
    Serial.println(motec.throttlePosition);

    Serial.print("CVT pressure raw: ");
    Serial.println(motec.cvtPressure);

    Serial.println();
    Serial.println(
        "CVT pressure scaling is still unverified."
    );

    Serial.println(
        "======================================="
    );

    Serial.println();
}

/////////////////////////// Interrupt Service Routines ///////////////////////////

void PR_RPM_ISR() {
    const uint32_t now = micros();

    v_PR_Interval_us =
        now - v_PR_Last_Pulse;

    v_PR_Last_Pulse = now;
    v_PR_Pulse_Available = true;
}

void RW_RPM_ISR() {
    const uint32_t now = micros();

    v_RW_Interval_us =
        now - v_RW_Last_Pulse;

    v_RW_Last_Pulse = now;
    v_RW_Pulse_Available = true;
}