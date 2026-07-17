#include "ecat_motor_master.h"
#include "ipe_joint_units.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

namespace {

constexpr uint32_t kExpectedVendor = 0x00041101;
constexpr uint32_t kExpectedProduct = 0x00009253;
constexpr int32_t kMaximumMoveCounts = 1000;
constexpr int32_t kMaximumFollowingError = 2000;
constexpr int kMinimumSpeedCountsPerSecond = 100;
constexpr int kMaximumSpeedCountsPerSecond = 1000;
constexpr int kDefaultSpeedCountsPerSecond = 1000;
constexpr double kAccelerationCountsPerSecondSquared = 2000.0;
constexpr int32_t kMaximumCsvCommandRaw = 10;
constexpr int32_t kMaximumCstCommandRaw = 2;
constexpr int32_t kMaximumCsvTravelCounts = 1000;
constexpr int32_t kMaximumCstTravelCounts = 250;

int expected_mode_display(const std::string& drive_mode) {
    if (drive_mode == "csp")
        return 8;
    if (drive_mode == "csv")
        return 9;
    if (drive_mode == "cst")
        return 10;
    return 0;
}

volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int) {
    stop_requested = 1;
}

using ipe::joint_units::counts_to_degrees;
using ipe::joint_units::degrees_to_counts;

template <typename T>
bool print_sdo_value(uint16_t index, uint8_t subindex, const char* name) {
    T value{};
    int size = sizeof(value);
    const int ret = ecatm_read_sdo(0, index, subindex, &value, &size);
    if (ret < 0 || size != static_cast<int>(sizeof(value))) {
        std::cout << name << " (0x" << std::hex << index << ':'
                  << static_cast<int>(subindex) << std::dec
                  << ")=READ_FAILED\n";
        return false;
    }
    std::cout << name << " (0x" << std::hex << index << ':'
              << static_cast<int>(subindex) << std::dec << ")="
              << static_cast<long long>(value) << '\n';
    return true;
}

int info() {
    std::cout << "Read-only SDO parameters (no reset, enable, or motion):\n";
    bool ok = true;
    ok &= print_sdo_value<int32_t>(0x2058, 0x00, "position_scaler");
    ok &= print_sdo_value<uint32_t>(0x2081, 0x00, "abn_encoder_steps");
    ok &= print_sdo_value<uint8_t>(0x3005, 0x01, "reduction_ratio");
    ok &= print_sdo_value<uint8_t>(0x3005, 0x02, "single_turn_absolute");
    ok &= print_sdo_value<uint8_t>(0x3005, 0x03, "dual_encoder");
    ok &= print_sdo_value<uint16_t>(0x280b, 0x00, "watchdog_time_ms");
    ok &= print_sdo_value<uint8_t>(0x280c, 0x00, "watchdog_enabled");
    return ok ? 0 : 6;
}

void read_and_print_state() {
    int32_t position = 0;
    int32_t target = 0;
    int32_t demand = 0;
    int32_t velocity = 0;
    int32_t torque = 0;
    uint16_t status = 0;
    uint16_t controlword = 0;
    int8_t mode_display = 0;
    uint16_t ethercat_state = 0;
    uint16_t al_status = 0;
    uint32_t error = 0;
    int actual_wkc = 0;
    int required_wkc = 0;
    esc_get_states(&position, &velocity, &torque);
    esc_get_status_word(&status);
    esc_get_error_codes(&error);
    const int csp_ret = ecatm_get_csp_diagnostics(0, &target, &demand,
                                                   nullptr, &controlword,
                                                   &mode_display);
    const int bus_ret = ecatm_get_bus_status(&ethercat_state, &al_status,
                                              &actual_wkc, &required_wkc);

    std::cout << "position=" << position
              << " (" << std::fixed << std::setprecision(3)
              << counts_to_degrees(position) << " deg)"
              << " velocity=" << velocity
              << " torque_raw=" << torque
              << " target=" << (csp_ret == 0 ? target : 0)
              << " demand=" << (csp_ret == 0 ? demand : 0)
              << " cw=0x" << std::hex << controlword << std::dec
              << " mode=" << static_cast<int>(mode_display)
              << " status=0x" << std::hex << std::setw(4)
              << std::setfill('0') << status << std::dec << std::setfill(' ')
              << " [" << ecatm_status_string(status) << "]"
              << " error=0x" << std::hex << std::setw(8)
              << std::setfill('0') << error << std::dec << std::setfill(' ')
              << " ecat=";
    if (bus_ret < 0) {
        std::cout << "UNKNOWN";
    } else {
        std::cout << (ethercat_state == EC_STATE_OPERATIONAL ? "OP" : "NOT_OP")
                  << "(0x" << std::hex << ethercat_state << ")"
                  << " AL=0x" << al_status << std::dec
                  << " wkc=" << actual_wkc << '/' << required_wkc;
    }
    std::cout << " pdo=" << (ecatm_is_pdo_healthy() ? "OK" : "ERROR")
              << '\n';
}

int probe_mode(const std::string& drive_mode) {
    const int expected = expected_mode_display(drive_mode);
    std::cout << "Mode probe: " << drive_mode
              << " (expected 0x6061=" << expected
              << "). No reset or enable; velocity/torque targets stay at zero.\n";

    /* Let several 1 ms PDO cycles carry 0x6060 to the drive and return 0x6061. */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int8_t displayed = 0;
    uint16_t status = 0;
    uint32_t error = 0;
    uint16_t ethercat_state = 0;
    uint16_t al_status = 0;
    int actual_wkc = 0;
    int required_wkc = 0;
    esc_get_status_word(&status);
    esc_get_error_codes(&error);
    const int diagnostic_ret = ecatm_get_csp_diagnostics(
        0, nullptr, nullptr, nullptr, nullptr, &displayed);
    const int bus_ret = ecatm_get_bus_status(
        &ethercat_state, &al_status, &actual_wkc, &required_wkc);

    read_and_print_state();
    const bool mode_ok = diagnostic_ret == 0 && displayed == expected;
    const bool bus_ok = bus_ret == 0 && ethercat_state == EC_STATE_OPERATIONAL &&
                        actual_wkc >= required_wkc && ecatm_is_pdo_healthy();
    const bool not_enabled = (status & 0x006f) != 0x0027;
    const bool pass = mode_ok && bus_ok && not_enabled;

    std::cout << "probe_result=" << (pass ? "PASS" : "FAIL")
              << " mode=" << static_cast<int>(displayed) << '/' << expected
              << " bus=" << (bus_ok ? "OK" : "ERROR")
              << " enabled=" << (not_enabled ? "NO" : "YES")
              << " error=0x" << std::hex << error << std::dec << '\n';
    if (!not_enabled)
        std::cerr << "Safety check failed: the drive entered Operation enabled during a passive probe.\n";
    return pass ? 0 : 7;
}

bool confirm_enable(const std::string& action) {
    std::cout << "\nThe joint will be enabled to perform: " << action << "\n"
              << "Secure the joint, remove the output load, clear the area, then type ENABLE: ";
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "ENABLE";
}

bool confirm_reset() {
    std::cout << "\nA standard CiA 402 Fault Reset will be sent. This does not enable the motor.\n"
              << "Type RESET to continue: ";
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "RESET";
}

bool clear_fault_for_motion() {
    uint16_t status = 0;
    uint32_t error = 0;
    esc_get_status_word(&status);
    esc_get_error_codes(&error);
    if ((status & 0x006f) != 0x0008)
        return true;

    std::cout << "The drive has a latched fault: status=0x" << std::hex << status
              << " error=0x" << error << std::dec << ".\n";
    if (!confirm_reset())
        return false;

    int ret = ecatm_fault_reset();
    read_and_print_state();
    if (ret < 0) {
        std::cerr << "Fault reset failed; enable is prohibited.\n";
        return false;
    }
    return true;
}

int monitor() {
    std::cout << "Passive monitor mode: the motor will not be enabled. Press Ctrl+C to exit.\n";
    while (!stop_requested) {
        read_and_print_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}

int reset_fault() {
    std::cout << "State before reset:\n";
    read_and_print_state();
    if (!confirm_reset()) {
        std::cout << "Not confirmed; no reset command was sent.\n";
        return 0;
    }

    int ret = ecatm_fault_reset();
    std::cout << "State after reset:\n";
    read_and_print_state();
    if (ret < 0) {
        std::cerr << "The fault remains, error code " << ret
                  << "; hold/move commands are prohibited.\n";
        return ret;
    }
    std::cout << "Fault Reset complete; the motor remains disabled.\n";
    return 0;
}

int enable_at_current_position(int32_t& origin) {
    int32_t velocity = 0;
    int32_t torque = 0;
    esc_get_states(&origin, &velocity, &torque);

    /* Prime CSP with the measured position before applying motor torque. */
    int32_t command = origin;
    ecatm_control(&command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return ecatm_enable();
}

int hold() {
    if (!clear_fault_for_motion())
        return 5;
    if (!confirm_enable("CSP hold at the current position")) {
        std::cout << "Not confirmed; remaining passive and exiting.\n";
        return 0;
    }

    int32_t origin = 0;
    int ret = enable_at_current_position(origin);
    if (ret < 0) {
        std::cerr << "Joint enable failed, error code " << ret << ".\n";
        return ret;
    }

    std::cout << "CSP hold active at " << origin << " count. Press Ctrl+C to stop.\n";
    while (!stop_requested) {
        ecatm_control(&origin);
        read_and_print_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}

bool following_error_is_safe(int32_t command) {
    int32_t actual = 0;
    int32_t velocity = 0;
    int32_t torque = 0;
    esc_get_states(&actual, &velocity, &torque);
    int64_t error = static_cast<int64_t>(command) - actual;
    if (std::llabs(error) > kMaximumFollowingError) {
        std::cerr << "Following error " << error
                  << " count exceeds safety limit; stopping.\n";
        return false;
    }
    return ecatm_is_pdo_healthy();
}

bool ramp_position(int32_t from, int32_t to,
                   int max_speed_counts_per_second =
                       kDefaultSpeedCountsPerSecond) {
    if (from == to)
        return true;
    if (max_speed_counts_per_second < kMinimumSpeedCountsPerSecond ||
        max_speed_counts_per_second > kMaximumSpeedCountsPerSecond) {
        std::cerr << "Invalid motion speed.\n";
        return false;
    }
    if (ecatm_arm_csp_motion() < 0) {
        std::cerr << "Cannot arm IPE CSP motion trigger.\n";
        return false;
    }
    std::cout << "csp_trigger=armed(cw=0x0f)\n";

    const int32_t direction = (to >= from) ? 1 : -1;
    const int64_t distance = std::llabs(static_cast<int64_t>(to) - from);
    const double requested_speed = max_speed_counts_per_second;
    const double acceleration = kAccelerationCountsPerSecondSquared;
    double peak_speed = requested_speed;
    double acceleration_time = peak_speed / acceleration;
    double acceleration_distance =
        0.5 * acceleration * acceleration_time * acceleration_time;
    double cruise_time = 0.0;
    if (2.0 * acceleration_distance > static_cast<double>(distance)) {
        peak_speed = std::sqrt(static_cast<double>(distance) * acceleration);
        acceleration_time = peak_speed / acceleration;
        acceleration_distance = 0.5 * static_cast<double>(distance);
    } else {
        cruise_time = (static_cast<double>(distance) -
                       2.0 * acceleration_distance) / peak_speed;
    }
    const double total_time = 2.0 * acceleration_time + cruise_time;

    std::cout << "profile=trapezoid distance=" << distance
              << " vmax=" << max_speed_counts_per_second
              << " accel=" << static_cast<int>(acceleration)
              << " peak=" << std::fixed << std::setprecision(1) << peak_speed
              << " estimated_time=" << std::setprecision(3) << total_time
              << "s\n";

    int32_t command = from;
    int check_counter = 0;
    int telemetry_counter = 0;
    bool motion_committed = false;
    const auto start = std::chrono::steady_clock::now();
    while (!stop_requested) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count();
        double travelled = 0.0;
        if (elapsed < acceleration_time) {
            travelled = 0.5 * acceleration * elapsed * elapsed;
        } else if (elapsed < acceleration_time + cruise_time) {
            travelled = acceleration_distance +
                        peak_speed * (elapsed - acceleration_time);
        } else if (elapsed < total_time) {
            const double remaining = total_time - elapsed;
            travelled = static_cast<double>(distance) -
                        0.5 * acceleration * remaining * remaining;
        } else {
            travelled = static_cast<double>(distance);
        }

        int64_t progress = static_cast<int64_t>(std::llround(travelled));
        if (progress > distance)
            progress = distance;
        const int32_t next_command = static_cast<int32_t>(
            static_cast<int64_t>(from) + direction * progress);
        if (next_command != command) {
            command = next_command;
            ecatm_control(&command);
            if (!motion_committed) {
                if (ecatm_commit_csp_motion() < 0) {
                    std::cerr << "Cannot commit IPE CSP motion trigger.\n";
                    return false;
                }
                std::cout << "csp_trigger=committed(cw=0x1f)\n";
                motion_committed = true;
            }
        }
        if (++check_counter >= 10) {
            check_counter = 0;
            if (!following_error_is_safe(command))
                return false;
        }
        if (++telemetry_counter >= 50) {
            telemetry_counter = 0;
            int32_t actual = 0;
            int32_t velocity = 0;
            int32_t torque = 0;
            esc_get_states(&actual, &velocity, &torque);
            std::cout << "phase=ramp command=" << command
                      << " actual=" << actual
                      << " error=" << (static_cast<int64_t>(command) - actual)
                      << " velocity_raw=" << velocity
                      << " torque_raw=" << torque << '\n';
        }
        if (progress >= distance)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !stop_requested && motion_committed;
}

bool hold_and_observe(const char* phase, int32_t command, int32_t origin,
                      std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    int print_divider = 0;
    while (std::chrono::steady_clock::now() < deadline && !stop_requested) {
        ecatm_control(&command);
        int32_t actual = 0;
        int32_t velocity = 0;
        int32_t torque = 0;
        esc_get_states(&actual, &velocity, &torque);
        const int64_t following_error = static_cast<int64_t>(command) - actual;
        if (++print_divider >= 10) {
            print_divider = 0;
            std::cout << "phase=" << phase << " command=" << command
                      << " actual=" << actual
                      << " error=" << following_error
                      << " delta_from_start_deg=" << std::fixed
                      << std::setprecision(3)
                      << counts_to_degrees(actual - origin)
                      << '\n';
        }
        if (std::llabs(following_error) > kMaximumFollowingError ||
            !ecatm_is_pdo_healthy())
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return !stop_requested;
}

int move(int32_t delta) {
    if (delta == 0 || std::llabs(static_cast<long long>(delta)) > kMaximumMoveCounts) {
        std::cerr << "move delta must be between -" << kMaximumMoveCounts
                  << " and " << kMaximumMoveCounts << " count, excluding zero.\n";
        return 2;
    }

    if (!clear_fault_for_motion())
        return 5;

    std::ostringstream description;
    description << "move " << delta << " count from the current position (about "
                << std::fixed << std::setprecision(3)
                << counts_to_degrees(delta) << " deg), then return";
    if (!confirm_enable(description.str())) {
        std::cout << "Not confirmed; remaining passive and exiting.\n";
        return 0;
    }

    int32_t origin = 0;
    int ret = enable_at_current_position(origin);
    if (ret < 0) {
        std::cerr << "Joint enable failed, error code " << ret << ".\n";
        return ret;
    }

    const int64_t target_wide = static_cast<int64_t>(origin) + delta;
    if (target_wide < std::numeric_limits<int32_t>::min() ||
        target_wide > std::numeric_limits<int32_t>::max()) {
        std::cerr << "Target position would overflow int32.\n";
        return 2;
    }
    const int32_t target = static_cast<int32_t>(target_wide);
    std::cout << "origin=" << origin << " target=" << target << '\n';
    if (!ramp_position(origin, target))
        return 3;
    if (!hold_and_observe("target_hold", target, origin,
                          std::chrono::seconds(2)))
        return 3;
    if (!ramp_position(target, origin))
        return 4;
    if (!hold_and_observe("return_hold", origin, origin,
                          std::chrono::seconds(1)))
        return 4;
    read_and_print_state();
    return 0;
}

bool pulse_feedback_is_safe(int32_t origin, int32_t maximum_travel,
                            const char* mode_name, int32_t command,
                            int telemetry_cycle) {
    int32_t position = 0;
    int32_t velocity = 0;
    int32_t torque = 0;
    uint16_t status = 0;
    uint32_t error = 0;
    esc_get_states(&position, &velocity, &torque);
    esc_get_status_word(&status);
    esc_get_error_codes(&error);
    const int64_t travel = static_cast<int64_t>(position) - origin;

    if (telemetry_cycle % 100 == 0) {
        std::cout << "phase=" << mode_name
                  << " command_raw=" << command
                  << " position=" << position
                  << " travel=" << travel
                  << " travel_deg=" << std::fixed << std::setprecision(4)
                  << counts_to_degrees(static_cast<int32_t>(travel))
                  << " velocity_raw=" << velocity
                  << " torque_raw=" << torque
                  << " status=0x" << std::hex << status
                  << " error=0x" << error << std::dec << '\n';
    }

    if (!ecatm_is_pdo_healthy()) {
        std::cerr << mode_name << " PDO became unhealthy during the test; zeroing immediately.\n";
        return false;
    }
    if ((status & 0x006f) != 0x0027 || error != 0) {
        std::cerr << mode_name << " left Operation enabled or faulted during the test; "
                  << "zeroing immediately.\n";
        return false;
    }
    if (std::llabs(travel) > maximum_travel) {
        std::cerr << mode_name << " travel guard triggered at " << travel
                  << " count; limit is +/-" << maximum_travel << " count.\n";
        return false;
    }
    return true;
}

int cyclic_pulse(const std::string& drive_mode, int32_t target_raw) {
    const bool is_csv = drive_mode == "csv";
    const int32_t maximum_command =
        is_csv ? kMaximumCsvCommandRaw : kMaximumCstCommandRaw;
    const int32_t maximum_travel =
        is_csv ? kMaximumCsvTravelCounts : kMaximumCstTravelCounts;
    const int ramp_ms = is_csv ? 250 : 200;
    const int hold_ms = is_csv ? 500 : 300;
    const char* mode_name = is_csv ? "CSV" : "CST";

    if (target_raw == 0 ||
        std::llabs(static_cast<long long>(target_raw)) > maximum_command) {
        std::cerr << mode_name << " command_raw must be within ±"
                  << maximum_command << ", excluding zero.\n";
        return 2;
    }
    if (!clear_fault_for_motion())
        return 5;

    std::ostringstream description;
    description << mode_name << " short pulse, command_raw=" << target_raw
                << ", ramp " << ramp_ms << " ms + hold " << hold_ms
                << " ms + zero ramp " << ramp_ms << " ms";
    if (!confirm_enable(description.str())) {
        std::cout << "Not confirmed; remaining passive and exiting.\n";
        return 0;
    }

    int32_t origin = 0;
    int32_t velocity = 0;
    int32_t torque = 0;
    esc_get_states(&origin, &velocity, &torque);
    int32_t command = 0;
    ecatm_control(&command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int ret = ecatm_enable();
    if (ret < 0) {
        std::cerr << mode_name << " enable failed, error code " << ret << ".\n";
        return ret;
    }

    std::cout << mode_name << " enabled: origin=" << origin
              << ", travel guard +/-" << maximum_travel << " count (about +/-"
              << std::fixed << std::setprecision(3)
              << counts_to_degrees(maximum_travel) << " deg).\n";

    bool safe = true;
    int cycle = 0;
    const int total_ms = ramp_ms + hold_ms + ramp_ms;
    for (int elapsed_ms = 0;
         elapsed_ms <= total_ms && !stop_requested && safe;
         ++elapsed_ms) {
        if (elapsed_ms < ramp_ms) {
            command = static_cast<int32_t>(
                std::llround(static_cast<double>(target_raw) * elapsed_ms /
                             ramp_ms));
        } else if (elapsed_ms < ramp_ms + hold_ms) {
            command = target_raw;
        } else {
            const int remaining = total_ms - elapsed_ms;
            command = static_cast<int32_t>(
                std::llround(static_cast<double>(target_raw) * remaining /
                             ramp_ms));
        }
        ecatm_control(&command);
        safe = pulse_feedback_is_safe(origin, maximum_travel, mode_name,
                                      command, cycle++);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    /* Zero the cyclic command before removing voltage, including every abort path. */
    command = 0;
    for (int i = 0; i < 300 && ecatm_is_pdo_healthy(); ++i) {
        ecatm_control(&command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ecatm_disable();
    std::cout << mode_name << " command zeroed; motor disabled.\n";
    read_and_print_state();
    return safe && !stop_requested ? 0 : 8;
}

void print_cyclic_session_help(const std::string& drive_mode) {
    const bool is_csv = drive_mode == "csv";
    std::cout
        << "Available commands:\n"
        << "  status          Show position, velocity, torque, status word, and PDO health\n"
        << "  reset           Clear a fault without enabling\n"
        << "  pulse <raw>     Run a time-limited pulse, then zero and disable; range +/-"
        << (is_csv ? kMaximumCsvCommandRaw : kMaximumCstCommandRaw) << "\n"
        << "  help            Show this help\n"
        << "  quit            Exit and close EtherCAT\n";
}

int cyclic_session(const std::string& drive_mode) {
    const std::string prompt = drive_mode == "csv" ? "csv> " : "cst> ";
    std::cout << (drive_mode == "csv" ? "CSV" : "CST")
              << " persistent safe session started. Each pulse ends at zero and disables "
                 "the motor while keeping EtherCAT connected.\n";
    print_cyclic_session_help(drive_mode);
    read_and_print_state();

    std::string line;
    while (!stop_requested) {
        std::cout << prompt << std::flush;
        if (!std::getline(std::cin, line))
            break;
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command.empty())
            continue;

        if (command == "status") {
            read_and_print_state();
        } else if (command == "help") {
            print_cyclic_session_help(drive_mode);
        } else if (command == "reset") {
            reset_fault();
        } else if (command == "pulse") {
            long long raw = 0;
            std::string extra;
            if (!(input >> raw) || (input >> extra) ||
                raw < std::numeric_limits<int32_t>::min() ||
                raw > std::numeric_limits<int32_t>::max()) {
                std::cout << "Usage: pulse <raw>. Enter help for the allowed range.\n";
                continue;
            }
            const int pulse_ret = cyclic_pulse(
                drive_mode, static_cast<int32_t>(raw));
            if (pulse_ret != 0)
                std::cout << "Pulse did not complete, return code=" << pulse_ret
                          << "; the connection remains active. Run status first.\n";
        } else if (command == "quit" || command == "exit") {
            break;
        } else {
            std::cout << "Unknown command. Enter help for usage.\n";
        }
    }
    return 0;
}

void print_session_help() {
    std::cout
        << "Available commands:\n"
        << "  status          Show position, state, and error code\n"
        << "  reset           Explicitly clear a fault without enabling\n"
        << "  enable          Hold the current position in CSP; requires ENABLE\n"
        << "  speed <100-1000> Set the maximum speed for later moves in count/s\n"
        << "  speed_deg <deg/s> Set speed in scaled degrees; still limited to 100-1000 count/s\n"
        << "  move <count>    Relative move; cumulative range is origin +/-1000 count\n"
        << "  move_deg <deg>  Relative move in scaled degrees with the same cumulative limit\n"
        << "  return          Return to the session origin\n"
        << "  disable         Disable the motor while keeping EtherCAT connected\n"
        << "  help            Show this help\n"
        << "  quit            Exit safely\n";
}

int session() {
    bool enabled = false;
    bool origin_set = false;
    int32_t session_origin = 0;
    int32_t command_position = 0;
    int session_speed = kDefaultSpeedCountsPerSecond;

    std::cout << "Persistent learning session started; EtherCAT will remain connected.\n";
    print_session_help();
    read_and_print_state();

    std::string line;
    while (!stop_requested) {
        std::cout << "ipe> " << std::flush;
        if (!std::getline(std::cin, line))
            break;
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command.empty())
            continue;

        if (command == "status") {
            read_and_print_state();
        } else if (command == "help") {
            print_session_help();
        } else if (command == "reset") {
            if (enabled) {
                std::cout << "Run disable before reset.\n";
                continue;
            }
            if (!confirm_reset())
                continue;
            int ret = ecatm_fault_reset();
            read_and_print_state();
            if (ret < 0)
                std::cout << "The fault remains active.\n";
        } else if (command == "enable") {
            if (enabled) {
                std::cout << "The joint is already enabled.\n";
                continue;
            }
            if (!clear_fault_for_motion())
                continue;
            if (!confirm_enable("persistent CSP hold"))
                continue;
            int32_t origin = 0;
            int ret = enable_at_current_position(origin);
            if (ret < 0) {
                std::cout << "Enable failed, error code " << ret << ".\n";
                continue;
            }
            enabled = true;
            /* A disable/enable cycle starts a fresh mechanically safe origin;
             * never reuse a target established under an earlier enable. */
            session_origin = origin;
            origin_set = true;
            command_position = origin;
            std::cout << "Enabled. session_origin=" << session_origin << '\n';
            read_and_print_state();
        } else if (command == "speed" || command == "speed_deg") {
            int64_t requested_speed_counts = 0;
            std::string extra;
            if (command == "speed") {
                long long requested_speed = 0;
                if (!(input >> requested_speed) || (input >> extra)) {
                    std::cout << "Usage: speed <100-1000>, in count/s.\n";
                    continue;
                }
                requested_speed_counts = requested_speed;
            } else {
                double requested_degrees_per_second = 0.0;
                if (!(input >> requested_degrees_per_second) || (input >> extra) ||
                    !std::isfinite(requested_degrees_per_second) ||
                    requested_degrees_per_second <= 0.0 ||
                    requested_degrees_per_second >
                        counts_to_degrees(kMaximumSpeedCountsPerSecond)) {
                    std::cout << "Usage: speed_deg <deg/s>, for example speed_deg 0.5.\n";
                    continue;
                }
                requested_speed_counts =
                    degrees_to_counts(requested_degrees_per_second);
                std::cout << "speed_deg=" << std::fixed << std::setprecision(6)
                          << requested_degrees_per_second << "°/s -> "
                          << requested_speed_counts << " count/s (actual scaled speed about "
                          << std::setprecision(6)
                          << counts_to_degrees(
                                 static_cast<int32_t>(requested_speed_counts))
                          << " deg/s)\n";
            }
            if (requested_speed_counts < kMinimumSpeedCountsPerSecond ||
                requested_speed_counts > kMaximumSpeedCountsPerSecond) {
                std::cout << "Rejected: converted speed must be 100-1000 count/s (about "
                          << std::fixed << std::setprecision(3)
                          << counts_to_degrees(kMinimumSpeedCountsPerSecond)
                          << " to "
                          << counts_to_degrees(kMaximumSpeedCountsPerSecond)
                          << " deg/s).\n";
                continue;
            }
            session_speed = static_cast<int>(requested_speed_counts);
            std::cout << "Maximum speed for later moves=" << session_speed
                      << " count/s.\n";
        } else if (command == "move" || command == "move_deg") {
            int64_t delta_counts = 0;
            std::string extra;
            if (command == "move") {
                long long requested_counts = 0;
                if (!(input >> requested_counts) || (input >> extra) ||
                    requested_counts == 0 ||
                    std::llabs(requested_counts) > kMaximumMoveCounts) {
                    std::cout << "Usage: move <count>; each move must be nonzero and within +/-1000.\n";
                    continue;
                }
                delta_counts = requested_counts;
            } else {
                double requested_degrees = 0.0;
                if (!(input >> requested_degrees) || (input >> extra) ||
                    !std::isfinite(requested_degrees) ||
                    std::fabs(requested_degrees) >
                        counts_to_degrees(kMaximumMoveCounts)) {
                    std::cout << "Usage: move_deg <deg>, for example move_deg 0.5.\n";
                    continue;
                }
                delta_counts = degrees_to_counts(requested_degrees);
                if (delta_counts == 0 ||
                    std::llabs(delta_counts) > kMaximumMoveCounts) {
                    std::cout << "Rejected: converted angle must be within +/-1 to +/-1000 count (about +/-"
                              << std::fixed << std::setprecision(3)
                              << counts_to_degrees(kMaximumMoveCounts)
                              << " deg).\n";
                    continue;
                }
                std::cout << "move_deg=" << std::fixed << std::setprecision(6)
                          << requested_degrees << "° -> " << delta_counts
                          << " count (actual scaled increment about "
                          << std::setprecision(6)
                          << counts_to_degrees(static_cast<int32_t>(delta_counts))
                          << " deg)\n";
            }
            if (!enabled || !origin_set) {
                std::cout << "Run enable first.\n";
                continue;
            }
            int64_t target_wide =
                static_cast<int64_t>(command_position) + delta_counts;
            if (std::llabs(target_wide - session_origin) > kMaximumMoveCounts) {
                std::cout << "Rejected: target exceeds session origin +/-1000 count.\n";
                continue;
            }
            int32_t target = static_cast<int32_t>(target_wide);
            if (!ramp_position(command_position, target, session_speed) ||
                !hold_and_observe("session_hold", target, session_origin,
                                  std::chrono::milliseconds(600))) {
                std::cout << "Motion aborted; disabling the motor.\n";
                ecatm_disable();
                enabled = false;
                continue;
            }
            command_position = target;
            read_and_print_state();
        } else if (command == "return") {
            if (!enabled || !origin_set) {
                std::cout << "There is no enabled session origin.\n";
                continue;
            }
            if (!ramp_position(command_position, session_origin, session_speed) ||
                !hold_and_observe("session_return", session_origin,
                                  session_origin, std::chrono::milliseconds(600))) {
                std::cout << "Return aborted; disabling the motor.\n";
                ecatm_disable();
                enabled = false;
                continue;
            }
            command_position = session_origin;
            read_and_print_state();
        } else if (command == "disable") {
            ecatm_disable();
            enabled = false;
            std::cout << "Motor disabled; EtherCAT remains connected.\n";
            read_and_print_state();
        } else if (command == "quit" || command == "exit") {
            break;
        } else {
            std::cout << "Unknown command. Enter help for usage.\n";
        }
    }

    if (enabled)
        ecatm_disable();
    return 0;
}

void print_usage(const char* program) {
    std::cerr << "Usage:\n"
              << "  sudo " << program
              << " [--interface enp130s0] mode_probe <csp|csv|cst>\n"
              << "  sudo " << program
              << " [--interface enp130s0] csv_pulse <raw:-10..-1|1..10>\n"
              << "  sudo " << program
              << " [--interface enp130s0] cst_pulse <raw:-2|-1|1|2>\n"
              << "  sudo " << program
              << " [--interface enp130s0] csv_session\n"
              << "  sudo " << program
              << " [--interface enp130s0] cst_session\n"
              << "  sudo " << program << " [--interface enp130s0] info\n"
              << "  sudo " << program << " [--interface enp130s0] monitor\n"
              << "  sudo " << program << " [--interface enp130s0] reset\n"
              << "  sudo " << program << " [--interface enp130s0] hold\n"
              << "  sudo " << program << " [--interface enp130s0] move [delta_counts]\n";
    std::cerr << "  sudo " << program << " [--interface enp130s0] session\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::string interface = "enp130s0";
    int argument = 1;
    if (argc >= 4 && std::string(argv[1]) == "--interface") {
        interface = argv[2];
        argument = 3;
    }
    if (argument >= argc) {
        print_usage(argv[0]);
        return 2;
    }

    const std::string action = argv[argument++];
    std::string drive_mode = "csp";
    int32_t delta = 100;
    if (action == "mode_probe") {
        if (argument >= argc) {
            print_usage(argv[0]);
            return 2;
        }
        drive_mode = argv[argument++];
        if (expected_mode_display(drive_mode) == 0 || argument != argc) {
            std::cerr << "mode_probe requires exactly one mode: csp, csv, or cst.\n";
            return 2;
        }
    } else if (action == "csv_session" || action == "cst_session") {
        drive_mode = action == "csv_session" ? "csv" : "cst";
    } else if (action == "csv_pulse" || action == "cst_pulse") {
        drive_mode = action == "csv_pulse" ? "csv" : "cst";
        if (argument >= argc) {
            print_usage(argv[0]);
            return 2;
        }
        char* end = nullptr;
        long parsed = std::strtol(argv[argument++], &end, 10);
        if (!end || *end != '\0' || parsed < std::numeric_limits<int32_t>::min() ||
            parsed > std::numeric_limits<int32_t>::max()) {
            std::cerr << "Invalid raw pulse command.\n";
            return 2;
        }
        delta = static_cast<int32_t>(parsed);
    } else if (action == "move" && argument < argc) {
        char* end = nullptr;
        long parsed = std::strtol(argv[argument++], &end, 10);
        if (!end || *end != '\0' || parsed < std::numeric_limits<int32_t>::min() ||
            parsed > std::numeric_limits<int32_t>::max()) {
            std::cerr << "Invalid delta_counts.\n";
            return 2;
        }
        delta = static_cast<int32_t>(parsed);
    } else if (action != "info" && action != "monitor" && action != "reset" &&
               action != "session" && action != "hold" && action != "move") {
        print_usage(argv[0]);
        return 2;
    }
    if (argument != argc) {
        print_usage(argv[0]);
        return 2;
    }

    if (ecatm_set_interface(interface.c_str()) < 0) {
        std::cerr << "Invalid or unavailable interface setting: " << interface << '\n';
        return 2;
    }

    int ret = ecatm_init_passive(drive_mode.c_str(), 1);
    if (ret < 0) {
        std::cerr << "Passive EtherCAT initialization failed: " << ret << '\n';
        return 1;
    }

    uint32_t vendor = 0;
    uint32_t product = 0;
    uint32_t revision = 0;
    ret = ecatm_get_slave_identity(0, &vendor, &product, &revision);
    if (ret < 0 || vendor != kExpectedVendor || product != kExpectedProduct) {
        std::cerr << "Unexpected slave identity: vendor=0x" << std::hex << vendor
                  << " product=0x" << product << " revision=0x" << revision
                  << std::dec << ". Motor enable is blocked.\n";
        ecatm_stop();
        return 1;
    }

    std::cout << "Verified IPE joint: vendor=0x" << std::hex << vendor
              << " product=0x" << product << " revision=0x" << revision
              << std::dec << '\n';

    if (action == "mode_probe")
        ret = probe_mode(drive_mode);
    else if (action == "csv_session" || action == "cst_session")
        ret = cyclic_session(drive_mode);
    else if (action == "csv_pulse" || action == "cst_pulse")
        ret = cyclic_pulse(drive_mode, delta);
    else if (action == "info")
        ret = info();
    else if (action == "monitor")
        ret = monitor();
    else if (action == "reset")
        ret = reset_fault();
    else if (action == "session")
        ret = session();
    else if (action == "hold")
        ret = hold();
    else
        ret = move(delta);

    std::cout << "Stopping drive and returning EtherCAT to SAFE-OP...\n";
    ecatm_stop();
    return ret < 0 ? 1 : ret;
}
