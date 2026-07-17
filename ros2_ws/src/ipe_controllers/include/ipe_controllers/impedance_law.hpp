#ifndef IPE_CONTROLLERS__IMPEDANCE_LAW_HPP_
#define IPE_CONTROLLERS__IMPEDANCE_LAW_HPP_

#include <algorithm>
#include <cmath>

namespace ipe_controllers {

inline double wrapped_position_error(double reference, double actual) {
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::remainder(reference - actual, kTwoPi);
}

inline double impedance_command_raw(
    double position_reference, double position, double velocity_reference,
    double velocity, double feedforward_raw, double kp_raw_per_rad,
    double kd_raw_per_rad_s, double breakaway_raw,
    double position_deadband_rad, double breakaway_velocity_threshold_rad_s,
    double command_limit_raw) {
  const double position_error = wrapped_position_error(position_reference, position);
  double command =
      kp_raw_per_rad * position_error +
      kd_raw_per_rad_s * (velocity_reference - velocity) + feedforward_raw;
  constexpr double kVelocityDeadbandRadS = 1e-4;
  if (breakaway_raw > 0.0 &&
      (std::abs(position_error) > position_deadband_rad ||
       std::abs(velocity_reference) > kVelocityDeadbandRadS) &&
      std::abs(velocity) < breakaway_velocity_threshold_rad_s &&
      std::abs(command) > 1e-9) {
    // Follow the feedback controller's requested effort direction. This lets
    // damping reverse the command during overspeed instead of continuously
    // accelerating in the trajectory direction.
    command += std::copysign(breakaway_raw, command);
  }
  return std::clamp(command, -command_limit_raw, command_limit_raw);
}

}  // namespace ipe_controllers

#endif  // IPE_CONTROLLERS__IMPEDANCE_LAW_HPP_
