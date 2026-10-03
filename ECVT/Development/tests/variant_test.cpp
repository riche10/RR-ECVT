// Exercises the real firmware source with mocked Arduino/CAN I/O on a PC.
// It does not emulate a Teensy, electrical signals, CAN delivery, or mechanics.
#include <iostream>
#include <stdexcept>
#include <main.cpp>

void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
#if !EXPECT_AUTOMATIC
        expect(!terminal.automaticControlArmed, "Bench must start disarmed");
#endif
        // No USB connection: startup must still reach the control loop.
        setup();
        expect(mockPwm == config.pwmNeutral, "Setup must initialize neutral");
        loop();
        expect(mockPwm == (EXPECT_AUTOMATIC ? config.pwmForwardMax : config.pwmNeutral),
               "First-loop output does not match selected variant");
        expect(mockCanFrames == 1 && mockLastCan.id == 0x130,
               "Expected periodic TCU telemetry");
        expect((uint16_t(mockLastCan.x[6]) << 8 | mockLastCan.x[7]) == config.pwmForwardMax,
               "Existing calculated-PWM CAN payload changed");

        Serial.connected = true;
#if EXPECT_AUTOMATIC
        const auto beforeCommands = mockWrites.size();
        const auto beforeTime = mockNowUs;
        Serial.inject("dnaf r");
        loop();
        expect(mockPwm == config.pwmForwardMax && mockWrites.size() == beforeCommands + 1,
               "Automatic must ignore bench commands and keep normal loop output");
        expect(mockNowUs == beforeTime, "Automatic must not execute blocking manual pulses");
#else
        Serial.inject("d\n");
        loop();
        expect(!terminal.automaticControlArmed && mockPwm == config.pwmNeutral,
               "d must disarm and command neutral in both variants");
        Serial.inject("a\n");
        loop();
        expect(terminal.automaticControlArmed && mockPwm == config.pwmForwardMax,
               "a must enable automatic control in both variants");
        Serial.inject("n\n");
        loop();
        expect(!terminal.automaticControlArmed && mockPwm == config.pwmNeutral,
               "n must disarm and command neutral in both variants");

        const auto beforePulse = mockWrites.size();
        Serial.inject("f");
        loop();
        expect(mockWrites.size() >= beforePulse + 2,
               "Disarmed manual test pulse missing");
        expect(mockWrites[beforePulse].second == config.pwmForwardStart,
               "Manual pulse value changed");
        expect(mockWrites[beforePulse + 1].second == config.pwmNeutral &&
               mockWrites[beforePulse + 1].first - mockWrites[beforePulse].first == 100000,
               "Manual pulse must return to neutral after 100 ms");
        Serial.inject("a");
        loop();
        const auto armedPulseStart = mockWrites.size();
        Serial.inject("r");
        loop();
        expect(mockWrites.size() == armedPulseStart + 1,
               "Armed controller must reject manual pulse");
#endif

        expect(chooseActiveTarget(77) == 2400 && chooseActiveTarget(78) == 3000,
               "RPM target boundary changed");
        expect(calculateMotorPwm(0, 2000) == config.pwmNeutral,
               "RPM deadband should hold neutral");
        expect(applySafetyClamp(config.pwmForwardMax, config.sheaveMin) == config.pwmNeutral,
               "Minimum position clamp changed");
        expect(applySafetyClamp(config.pwmReverseMax, config.sheaveMax) == config.pwmNeutral,
               "Maximum position clamp changed");
        expect(applySafetyClamp(4096, 2000) == config.pwmNeutral,
               "Invalid PWM clamp changed");
        motec.cvtPressure = 255;
        expect(applySafetyClamp(config.pwmForwardMax, 2000) == config.pwmForwardMax,
               "Pressure decoding/threshold behavior unexpectedly changed");

        // Both variants must produce identical controller results for this grid.
        uint64_t fingerprint = 1469598103934665603ULL;
        for (uint16_t pot : {uint16_t(1175), uint16_t(1468), uint16_t(2000),
                             uint16_t(3294), uint16_t(3660)}) {
            for (uint16_t primary : {uint16_t(0), uint16_t(2300), uint16_t(2400),
                                     uint16_t(2450), uint16_t(3000), uint16_t(3500)}) {
                for (uint16_t wheel : {uint16_t(0), uint16_t(77), uint16_t(78), uint16_t(200)}) {
                    SensorData sensors;
                    sensors.rawPotValue = pot;
                    sensors.primaryAvgRpm = primary;
                    sensors.rearWheelAvgRpm = wheel;
                    const auto result = calculateControl(sensors);
                    fingerprint ^= applySafetyClamp(result.calculatedPwm, pot);
                    fingerprint *= 1099511628211ULL;
                }
            }
        }
        std::cout << "PASS: startup, setup neutral, variant-specific serial behavior, CAN payload, limits\n";
        std::cout << "CONTROL_GRID_FINGERPRINT=" << fingerprint << "\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << "\n";
        return 1;
    }
}
