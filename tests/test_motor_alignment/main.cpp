/**
 * @file main.cpp
 * @brief Regress the measured scooter and JK42BLS01 Hall/commutation alignments
 */
#include "motor_pll.hpp"
#include <cstdio>
#include <initializer_list>
#include <type_traits>

// Accept both the runtime-clock baseline and the compile-time-clock PI branch.
template<class Pll>
Pll makePll(bool inverse) {
    if constexpr (std::is_constructible<Pll, float, float, bool>::value) {
        return Pll(20000.0f, 4800.0f, inverse);
    } else {
        return Pll(4800.0f, inverse);
    }
}

int main() {
    // These rows describe the measured motor/Hall wiring, not a generated
    // arithmetic identity: the inverse motor needs the opposite voltage vector.
    const uint8_t forward[2][6] = {{1, 2, 3, 4, 5, 0}, {4, 5, 0, 1, 2, 3}};
    int checks = 0;
    int failures = 0;
    auto check = [&](bool ok, const char* message) {
        ++checks;
        if (!ok) { ++failures; std::printf("FAIL: %s\n", message); }
    };
    for (int inverse = 0; inverse < 2; ++inverse) {
        auto pll = makePll<libecu::MotorPLL>(inverse != 0);
        for (uint8_t hall = 0; hall < 6; ++hall) {
            pll.updateHall(hall);
            pll.reset();
            for (bool use_pll : {false, true}) {
                pll.setUsePLL(use_pll);
                volatile libecu::DriveMode direction = libecu::DriveMode::FORWARD;
                check(pll.getNextHall(direction) == forward[inverse][hall],
                      "forward commutation matches the motor/Hall alignment");
                direction = libecu::DriveMode::REVERSE;
                check(pll.getNextHall(direction) == (forward[inverse][hall] + 3) % 6,
                      "reverse applies the opposite voltage vector in every Hall sector");
                direction = libecu::DriveMode::NEUTRAL;
                check(pll.getNextHall(direction) == hall, "neutral has no torque phase offset");
            }
        }
    }
    std::printf("Motor alignment: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
