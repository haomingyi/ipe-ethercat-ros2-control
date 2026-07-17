#ifndef IPE_JOINT_UNITS_HPP
#define IPE_JOINT_UNITS_HPP

#include <cmath>
#include <cstdint>
#include <limits>

namespace ipe::joint_units {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kCountsPerRevolution = 262144.0;

inline double counts_to_degrees(int64_t counts) {
    return static_cast<double>(counts) * 360.0 / kCountsPerRevolution;
}

inline int64_t degrees_to_counts(double degrees) {
    return static_cast<int64_t>(
        std::llround(degrees * kCountsPerRevolution / 360.0));
}

inline double counts_to_radians(int64_t counts) {
    return static_cast<double>(counts) * 2.0 * kPi /
           kCountsPerRevolution;
}

inline int64_t radians_to_counts(double radians) {
    return static_cast<int64_t>(
        std::llround(radians * kCountsPerRevolution / (2.0 * kPi)));
}

inline bool is_valid_direction(int direction) {
    return direction == 1 || direction == -1;
}

inline double position_to_radians(int32_t actual_count, int32_t zero_count,
                                  int direction) {
    if (!is_valid_direction(direction))
        return std::numeric_limits<double>::quiet_NaN();
    const int64_t relative_count =
        static_cast<int64_t>(actual_count) - zero_count;
    return direction * counts_to_radians(relative_count);
}

inline bool radians_to_position(double radians, int32_t zero_count,
                                int direction, int32_t* target_count) {
    if (!target_count || !std::isfinite(radians) ||
        !is_valid_direction(direction)) {
        return false;
    }

    const double raw_count =
        static_cast<double>(zero_count) +
        direction * radians * kCountsPerRevolution / (2.0 * kPi);
    if (!std::isfinite(raw_count) ||
        raw_count < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
        raw_count > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return false;
    }

    *target_count = static_cast<int32_t>(std::llround(raw_count));
    return true;
}

}  // namespace ipe::joint_units

#endif
