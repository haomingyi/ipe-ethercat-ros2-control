#include "ipe_controllers/impedance_law.hpp"

#include <cassert>
#include <cmath>

int main() {
  using ipe_controllers::impedance_command_raw;
  using ipe_controllers::wrapped_position_error;

  assert(std::abs(wrapped_position_error(0.2, 0.1) - 0.1) < 1e-12);
  assert(std::abs(wrapped_position_error(-3.13, 3.13) - 0.023185307179586) < 1e-9);
  assert(std::abs(impedance_command_raw(
      0.2, 0.1, 0.0, 0.0, 0.0, 20.0, 4.0, 0.0, 0.0, 0.02, 100.0) - 2.0) < 1e-12);
  assert(impedance_command_raw(
      2.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.02, 50.0) == 50.0);
  assert(impedance_command_raw(
      -2.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.02, 50.0) == -50.0);
  assert(std::abs(impedance_command_raw(
      0.2, 0.1, 0.0, 0.0, 0.0, 20.0, 4.0, 20.0, 0.01, 0.02, 100.0) - 22.0) < 1e-12);
  assert(std::abs(impedance_command_raw(
      -0.2, -0.1, 0.0, 0.0, 0.0, 20.0, 4.0, 20.0, 0.01, 0.02, 100.0) + 22.0) < 1e-12);
  assert(std::abs(impedance_command_raw(
      0.005, 0.0, 0.0, 0.0, 0.0, 20.0, 4.0, 20.0, 0.01, 0.02, 100.0) - 0.1) < 1e-12);
  assert(std::abs(impedance_command_raw(
      0.1, 0.0, 0.1, 1.0, 0.0, 20.0, 4.0, 20.0, 0.01, 0.02, 100.0) + 1.6) < 1e-12);
  return 0;
}
