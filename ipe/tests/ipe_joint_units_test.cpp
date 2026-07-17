#include "ipe_joint_units.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

bool near(double actual, double expected, double tolerance = 1e-12) {
    return std::fabs(actual - expected) <= tolerance;
}

int failures = 0;

void expect(bool condition, const char* description) {
    if (condition)
        return;
    std::cerr << "FAILED: " << description << '\n';
    ++failures;
}

}  // namespace

int main() {
    using namespace ipe::joint_units;

    expect(near(counts_to_degrees(262144), 360.0), "one revolution in degrees");
    expect(near(counts_to_radians(262144), 2.0 * kPi), "one revolution in radians");
    expect(degrees_to_counts(0.5) == 364, "0.5 degree rounds to 364 count");
    expect(radians_to_counts(kPi) == 131072, "pi radians is half a revolution");

    constexpr int32_t zero_count = 130336;
    constexpr int32_t measured_count = 130700;
    const double positive_angle =
        position_to_radians(measured_count, zero_count, 1);
    expect(near(positive_angle, counts_to_radians(364)), "positive direction conversion");
    expect(near(position_to_radians(measured_count, zero_count, -1),
                -positive_angle), "negative direction conversion");
    expect(std::isnan(position_to_radians(measured_count, zero_count, 0)),
           "invalid direction produces NaN");

    int32_t target_count = 0;
    expect(radians_to_position(positive_angle, zero_count, 1, &target_count),
           "positive direction inverse conversion succeeds");
    expect(target_count == measured_count, "positive inverse conversion value");
    expect(radians_to_position(-positive_angle, zero_count, -1, &target_count),
           "negative direction inverse conversion succeeds");
    expect(target_count == measured_count, "negative inverse conversion value");
    expect(!radians_to_position(positive_angle, zero_count, 0, &target_count),
           "invalid inverse direction is rejected");
    expect(!radians_to_position(positive_angle, zero_count, 1, nullptr),
           "null inverse output is rejected");

    if (failures != 0)
        return 1;

    std::cout << "IPE joint unit tests passed\n"
              << "1 count = " << counts_to_degrees(1) << " deg = "
              << counts_to_radians(1) << " rad\n"
              << "364 count = " << counts_to_degrees(364) << " deg = "
              << counts_to_radians(364) << " rad\n";
    return 0;
}
