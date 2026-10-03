#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

constexpr int INPUT = 0, OUTPUT = 1, CHANGE = 2, RISING = 3;
inline uint32_t mockNowUs = 3000000;
inline uint16_t mockPot = 2000;
inline uint16_t mockPwm = 0;
inline std::vector<std::pair<uint32_t, uint16_t>> mockWrites;

struct MockSerial {
    bool connected = false;
    std::deque<char> input;
    explicit operator bool() const { return connected; }
    void begin(uint32_t) {}
    int available() const { return static_cast<int>(input.size()); }
    char read() { auto c = input.front(); input.pop_front(); return c; }
    void inject(const std::string &text) { for (char c : text) input.push_back(c); }
    template<typename... T> void print(T...) {}
    template<typename... T> void println(T...) {}
};
inline MockSerial Serial;
inline uint32_t millis() { return mockNowUs / 1000; }
inline uint32_t micros() { return mockNowUs; }
inline void delay(uint32_t ms) { mockNowUs += ms * 1000; }
inline void pinMode(uint8_t, int) {}
inline void analogReadResolution(int) {}
inline void analogReadAveraging(int) {}
inline void analogWriteFrequency(uint8_t, int) {}
inline void analogWriteResolution(int) {}
inline uint16_t analogRead(uint8_t) { return mockPot; }
inline void analogWrite(uint8_t, uint16_t value) {
    mockPwm = value;
    mockWrites.emplace_back(mockNowUs, value);
}
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void noInterrupts() {}
inline void interrupts() {}
template<typename T> T constrain(T value, T low, T high) {
    return std::min(std::max(value, low), high);
}
inline long map(long value, long a, long b, long c, long d) {
    return (value - a) * (d - c) / (b - a) + c;
}
