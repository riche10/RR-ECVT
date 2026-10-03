#include <Arduino.h>
#include <MyCANBUS.h>

/*
    ECVT TCU Refactored Automatic Firmware
    -------------------------------------
    Omari Rich's automatic ECVT controller. After setup, the control loop
    runs without a GUI, serial connection, or arming command.

    Setup initializes neutral. Each loop then applies the calculated PWM
    after the existing position and PWM checks. Bench terminal commands,
    manual pulses, and the arming gate belong to the Bench Testing version.
    USB Serial provides periodic diagnostic output only.

    Pressure scaling, sensor edge counting, and actuator direction retain
    their existing verification notes in the configuration and control code.
*/

/////////////////////////// Firmware Information ///////////////////////////

constexpr char FIRMWARE_NAME[] = "ECVT TCU Refactor Automatic";
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


RpmState primaryRpm;
RpmState rearWheelRpm;

MotecData motec;


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
    uint16_t pwmOutput
);

void writeAutomaticMotorCommand(uint16_t pwmOutput);
void forceNeutral();


void printDebugData(
    const SensorData &sensors,
    const ControlData &control
);


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
        Serial.println("Automatic motor control starts after setup.");
        Serial.println("Serial output is diagnostic only; no terminal commands.");
    }
}

void setupSerial() {
    Serial.begin(config.serialBaud);

    // Automatic operation does not wait for a USB Serial connection.
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

    SensorData sensors = readSensors();

    receiveCanData();

    ControlData control = calculateControl(sensors);

    control.calculatedPwm = applySafetyClamp(
        control.calculatedPwm,
        sensors.rawPotValue
    );


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
    uint16_t pwmOutput
) {
    static uint32_t lastCanSendMs = 0;

    const uint32_t now = millis();

    if (
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

    analogWrite(
        Pins::MOTOR_PWM,
        pwmOutput
    );
}


/////////////////////////// Debug Printing ///////////////////////////

void printDebugData(
    const SensorData &sensors,
    const ControlData &control
) {
    if (!Serial) {
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

    Serial.print("Applied PWM command: ");
    Serial.println(control.calculatedPwm);

    Serial.println("-----------------------------------");
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
