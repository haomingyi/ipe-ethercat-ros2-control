#include <cstdint>
#include <pthread.h>

#include "cia402_def.h"
#include "ecat_motor_master.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>

extern "C" {
extern volatile uint8_t motor_mode;
extern pthread_mutex_t pdo_mutex;
}

namespace {

static_assert(sizeof(TCiA402PDO1600) == 20, "Unexpected IPE RxPDO layout");
static_assert(sizeof(TCiA402PDO1A00) == 32, "Unexpected IPE TxPDO layout");

constexpr uint32_t kVendor = 0x00041101u;
constexpr uint32_t kProduct = 0x00009253u;
constexpr int32_t kCountsPerRevolution = 262144;
constexpr int32_t kCspLimit = 1000;
constexpr double kCspLiveRatedRpm = 3000.0 / 101.0;
constexpr double kCspLiveMaxRelativeTurns = 100.0;
constexpr int32_t kCspLiveFollowingErrorLimit = 16384;
constexpr int32_t kCsvRawLimit = 5000;
/* A timed 200000-raw test produced about 163 output degrees/minute. Together
 * with the 18-bit output position change, this identifies an approximately
 * 101:1 relationship between 0x606C velocity raw and flange position. The
 * hard limit below maps 3000 motor rpm to raw counts per second. */
constexpr int32_t kCsvLiveRawLimit = 13107200;
constexpr int32_t kCstRawLimit = 10;
constexpr int32_t kCstLiveCautionBoundary = 100;
constexpr int32_t kCstLiveRawLimit = 300;
constexpr int32_t kCstLiveVelocityLimit = kCsvLiveRawLimit;
constexpr int32_t kCsvTravelLimit = 1000;
constexpr int32_t kCstTravelLimit = 500;
constexpr int32_t kCsvLiveTravelLimit = 0;  // 0: no limit; confirmed no internal cable.
/* 1000 raw/ms reaches or leaves the rated ceiling in about 13.1 seconds. */
constexpr int32_t kCsvLiveSlewPerMs = 1000;
constexpr uint8_t kMotorModeCsp = 0;
constexpr uint8_t kMotorModeCsv = 1;
constexpr uint8_t kMotorModeCst = 2;

volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int) {
    stop_requested = 1;
}

double counts_to_degrees(int64_t count) {
    return static_cast<double>(count) * 360.0 / kCountsPerRevolution;
}

struct Snapshot {
    int8_t mode_display{0};
    uint16_t status{0};
    int32_t position_demand{0};
    int32_t position{0};
    int32_t velocity_demand{0};
    int32_t velocity{0};
    int16_t torque_demand{0};
    int16_t torque{0};
    uint32_t error{0xffffffffu};
    int8_t mode_command{0};
    uint16_t controlword{0};
    int32_t target_position{0};
    int32_t target_velocity{0};
    int16_t target_torque{0};
};

bool read_snapshot(Snapshot& value) {
    if (ec_slavecount < 1)
        return false;
    pthread_mutex_lock(&pdo_mutex);
    auto* output = reinterpret_cast<TCiA402PDO1600*>(ec_slave[1].outputs);
    auto* input = reinterpret_cast<TCiA402PDO1A00*>(ec_slave[1].inputs);
    if (!output || !input) {
        pthread_mutex_unlock(&pdo_mutex);
        return false;
    }
    value.mode_display = input->ObjModesOfOperationDisplay;
    value.status = input->ObjStatusWord;
    value.position_demand = input->ObjPositionDemandValue;
    value.position = input->ObjPositionActualValue;
    value.velocity_demand = input->ObjVelocityDemandValue;
    value.velocity = input->ObjVelocityActualValue;
    value.torque_demand = input->ObjTorqueDemandValue;
    value.torque = input->ObjTorqueActualValue;
    value.error = input->ErrorCode;
    value.mode_command = output->ObjModesOfOperation;
    value.controlword = output->ObjControlWord;
    value.target_position = output->ObjTargetPosition;
    value.target_velocity = output->ObjTargetVelocity;
    value.target_torque = output->ObjTargetTorque;
    pthread_mutex_unlock(&pdo_mutex);
    return true;
}

void write_controlword(uint16_t value) {
    pthread_mutex_lock(&pdo_mutex);
    auto* output = reinterpret_cast<TCiA402PDO1600*>(ec_slave[1].outputs);
    if (output)
        output->ObjControlWord = value;
    pthread_mutex_unlock(&pdo_mutex);
}

void print_snapshot() {
    Snapshot s;
    uint16_t ethercat_state = 0;
    uint16_t al_status = 0;
    int wkc = 0;
    int expected_wkc = 0;
    const bool snapshot_ok = read_snapshot(s);
    const bool bus_ok = ecatm_get_bus_status(
        &ethercat_state, &al_status, &wkc, &expected_wkc) == 0;
    if (!snapshot_ok) {
        std::cout << "state=UNAVAILABLE\n";
        return;
    }
    std::cout << "mode=" << static_cast<int>(s.mode_display)
              << " cmd_mode=" << static_cast<int>(s.mode_command)
              << " status=0x" << std::hex << std::setw(4)
              << std::setfill('0') << s.status << std::dec << std::setfill(' ')
              << " [" << ecatm_status_string(s.status) << ']'
              << " error=0x" << std::hex << std::setw(8)
              << std::setfill('0') << s.error << std::dec << std::setfill(' ')
              << "\nposition=" << s.position
              << " position_deg=" << std::fixed << std::setprecision(4)
              << counts_to_degrees(s.position)
              << " target_position=" << s.target_position
              << " demand_position=" << s.position_demand
              << "\nvelocity_raw=" << s.velocity
              << " target_velocity_raw=" << s.target_velocity
              << " demand_velocity_raw=" << s.velocity_demand
              << "\ntorque_raw=" << s.torque
              << " target_torque_raw=" << s.target_torque
              << " demand_torque_raw=" << s.torque_demand
              << " cw=0x" << std::hex << s.controlword << std::dec
              << "\necat=";
    if (!bus_ok) {
        std::cout << "UNKNOWN";
    } else {
        std::cout << (ethercat_state == EC_STATE_OPERATIONAL ? "OP" : "NOT_OP")
                  << "(0x" << std::hex << ethercat_state << ')'
                  << " AL=0x" << al_status << std::dec
                  << " wkc=" << wkc << '/' << expected_wkc;
    }
    std::cout << " pdo=" << (ecatm_is_pdo_healthy() ? "OK" : "ERROR")
              << '\n';
}

bool confirm(const char* word, const std::string& description) {
    std::cout << description << "\nType " << word << " to continue: " << std::flush;
    std::string answer;
    return std::getline(std::cin, answer) && answer == word;
}

int expected_mode(const std::string& name) {
    if (name == "csp") return CYCLIC_SYNC_POSITION_MODE;
    if (name == "csv") return CYCLIC_SYNC_VELOCITY_MODE;
    if (name == "cst") return CYCLIC_SYNC_TORQUE_MODE;
    return 0;
}

uint8_t internal_mode(const std::string& name) {
    if (name == "csp") return kMotorModeCsp;
    if (name == "csv") return kMotorModeCsv;
    return kMotorModeCst;
}

bool select_mode(const std::string& name) {
    const int requested = expected_mode(name);
    if (requested == 0)
        return false;

    ecatm_disable();
    Snapshot before;
    if (!read_snapshot(before))
        return false;

    pthread_mutex_lock(&pdo_mutex);
    auto* output = reinterpret_cast<TCiA402PDO1600*>(ec_slave[1].outputs);
    output->ObjControlWord = CONTROLWORD_COMMAND_DISABLEVOLTAGE;
    output->ObjTargetPosition = before.position;
    output->ObjTargetVelocity = 0;
    output->ObjTargetTorque = 0;
    output->ObjModesOfOperation = static_cast<int8_t>(requested);
    motor_mode = internal_mode(name);
    pthread_mutex_unlock(&pdo_mutex);

    for (int attempt = 0; attempt < 100; ++attempt) {
        Snapshot after;
        if (read_snapshot(after) && after.mode_display == requested) {
            std::cout << "mode_switch=OK mode=" << name
                      << " display=" << requested << " enabled=NO\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cerr << "Mode switch failed: 0x6061 did not report " << requested << ".\n";
    return false;
}

bool clear_fault_if_needed() {
    Snapshot s;
    if (!read_snapshot(s))
        return false;
    if ((s.status & 0x006f) != STATUSWORD_STATE_FAULT)
        return true;
    std::ostringstream message;
    message << "Drive is in Fault: status=0x" << std::hex << s.status
            << " error=0x" << s.error << std::dec
            << ". A standard Fault Reset will be sent without enabling the drive.";
    if (!confirm("RESET", message.str()))
        return false;
    if (ecatm_fault_reset() < 0) {
        std::cerr << "Fault Reset failed.\n";
        return false;
    }
    print_snapshot();
    return true;
}

bool feedback_safe(int32_t origin, int32_t travel_limit, int cycle,
                   int print_every = 100, int32_t velocity_limit = 0) {
    Snapshot s;
    if (!read_snapshot(s))
        return false;
    const int64_t travel = static_cast<int64_t>(s.position) - origin;
    if (print_every > 0 && cycle % print_every == 0) {
        std::cout << "travel=" << travel
                  << " travel_deg=" << std::fixed << std::setprecision(4)
                  << counts_to_degrees(travel)
                  << " velocity_raw=" << s.velocity
                  << " torque_raw=" << s.torque
                  << " target_v_raw=" << s.target_velocity
                  << " demand_v_raw=" << s.velocity_demand
                  << " target_t_raw=" << s.target_torque
                  << " demand_t_raw=" << s.torque_demand
                  << " cw=0x" << std::hex << s.controlword << std::dec << '\n';
    }
    if (!ecatm_is_pdo_healthy()) {
        std::cerr << "PDO is unhealthy; zeroing immediately.\n";
        return false;
    }
    if ((s.status & 0x006f) != STATUSWORD_STATE_OPERATIONENABLED ||
        s.error != 0) {
        std::cerr << "Drive left Operation enabled or faulted; zeroing immediately.\n";
        return false;
    }
    if (travel_limit > 0 && std::llabs(travel) > travel_limit) {
        std::cerr << "Travel protection triggered at " << travel << " count; limit +/-"
                  << travel_limit << "。\n";
        return false;
    }
    if (velocity_limit > 0 &&
        std::llabs(static_cast<long long>(s.velocity)) > velocity_limit) {
        std::cerr << "Velocity protection triggered at " << s.velocity << " raw; limit +/-"
                  << velocity_limit << "; zeroing immediately.\n";
        return false;
    }
    return true;
}

void safe_command_and_disable() {
    int32_t safe_command = 0;
    if (motor_mode == kMotorModeCsp) {
        Snapshot current;
        if (read_snapshot(current))
            safe_command = current.position;
    }
    for (int i = 0; i < 300 && ecatm_is_pdo_healthy(); ++i) {
        ecatm_control(&safe_command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ecatm_disable();
}

bool csp_leg(int32_t from, int32_t to, int32_t safety_origin) {
    if (ecatm_arm_csp_motion() < 0)
        return false;
    int32_t command = from;
    const int32_t direction = to >= from ? 1 : -1;
    bool committed = false;
    int cycle = 0;
    while (command != to && !stop_requested) {
        command += direction;
        ecatm_control(&command);
        if (!committed) {
            if (ecatm_commit_csp_motion() < 0)
                return false;
            committed = true;
        }
        if (!feedback_safe(safety_origin, kCspLimit + 100, cycle++))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !stop_requested;
}

bool wait_for_csp_target(int32_t target, int32_t safety_origin,
                         int timeout_ms = 1500) {
    int stable_cycles = 0;
    for (int cycle = 0; cycle < timeout_ms && !stop_requested; ++cycle) {
        ecatm_control(&target);
        Snapshot s;
        if (!read_snapshot(s) ||
            !feedback_safe(safety_origin, kCspLimit + 100, cycle))
            return false;
        const int64_t following_error =
            static_cast<int64_t>(target) - s.position;
        if (cycle % 100 == 0)
            std::cout << "csp_wait target=" << target
                      << " actual=" << s.position
                      << " error=" << following_error << '\n';
        if (std::llabs(following_error) <= 8) {
            if (++stable_cycles >= 100)
                return true;
        } else {
            stable_cycles = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cerr << "Timed out waiting for CSP to settle; success will not be reported.\n";
    return false;
}

int run_csp(int32_t delta) {
    if (delta == 0 || std::llabs(static_cast<long long>(delta)) > kCspLimit) {
        std::cout << "CSP range is +/-" << kCspLimit << " count and cannot be zero.\n";
        return 2;
    }
    if (!clear_fault_if_needed())
        return 3;
    if (!confirm("ENABLE", "CSP will move by the requested count and return to the start."))
        return 0;

    Snapshot before;
    read_snapshot(before);
    int32_t command = before.position;
    ecatm_control(&command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        std::cerr << "CSP enable failed.\n";
        return 4;
    }
    Snapshot enabled;
    read_snapshot(enabled);
    const int32_t origin = enabled.position;
    const int32_t target = origin + delta;
    bool ok = csp_leg(origin, target, origin);
    if (ok)
        ok = wait_for_csp_target(target, origin);
    if (ok)
        ok = csp_leg(target, origin, origin);
    if (ok)
        ok = wait_for_csp_target(origin, origin);
    safe_command_and_disable();
    print_snapshot();
    return ok ? 0 : 5;
}

int run_csp_live() {
    if (!confirm(
            "NO_CABLE",
            "Multi-turn CSP is allowed only when no internal cable can be wound."))
        return 0;
    if (!clear_fault_if_needed())
        return 3;
    if (!confirm(
            "CSP_LIVE",
            "CSP live trajectory limit is about 29.7 flange rpm and +/-100 turns per command."))
        return 0;
    if (!confirm("ENABLE", "CSP live position control stays enabled; use stop to exit."))
        return 0;

    Snapshot before;
    if (!read_snapshot(before))
        return 4;
    int32_t applied = before.position;
    ecatm_control(&applied);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        std::cerr << "CSP live position enable failed.\n";
        return 4;
    }

    Snapshot enabled;
    if (!read_snapshot(enabled)) {
        safe_command_and_disable();
        return 5;
    }
    const int32_t origin = enabled.position;
    applied = origin;
    int32_t target = origin;
    double trajectory = static_cast<double>(origin);
    double speed_rpm = 1.0;
    bool moving = false;
    bool trigger_pending = false;
    bool ok = true;
    bool exit_requested = false;
    int cycle = 0;
    auto previous_time = std::chrono::steady_clock::now();

    std::cout
        << "CSP live control started at the current position; default speed is 1 rpm.\n"
        << "Commands: speed <rpm>, move_deg <relative-degrees>, turns <relative-turns>, "
           "hold，status，stop。\n"
        << "Rated speed range 0.1 to " << std::fixed << std::setprecision(3)
        << kCspLiveRatedRpm << " rpm; relative target limit +/-"
        << kCspLiveMaxRelativeTurns << " turns.\n"
        << "csp-live> " << std::flush;

    while (!stop_requested && ok && !exit_requested) {
        const auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - previous_time).count();
        previous_time = now;
        elapsed = std::clamp(elapsed, 0.0, 0.01);

        if (moving) {
            const double remaining = static_cast<double>(target) - trajectory;
            const double max_step = speed_rpm * kCountsPerRevolution / 60.0 * elapsed;
            if (std::abs(remaining) <= max_step) {
                trajectory = static_cast<double>(target);
                moving = false;
            } else {
                trajectory += remaining > 0.0 ? max_step : -max_step;
            }
            applied = static_cast<int32_t>(std::llround(trajectory));
        }
        ecatm_control(&applied);
        if (trigger_pending) {
            if (ecatm_commit_csp_motion() < 0) {
                std::cerr << "Failed to commit the CSP trigger edge; stopping immediately.\n";
                ok = false;
            }
            trigger_pending = false;
        }

        Snapshot state;
        if (!feedback_safe(origin, 0, cycle, 0) || !read_snapshot(state)) {
            ok = false;
        } else {
            const int64_t following =
                static_cast<int64_t>(applied) - state.position;
            if (std::llabs(following) > kCspLiveFollowingErrorLimit) {
                std::cerr << "CSP following error=" << following << " count exceeds +/-"
                          << kCspLiveFollowingErrorLimit << "; stopping immediately.\n";
                ok = false;
            }
        }
        ++cycle;

        pollfd input_fd{};
        input_fd.fd = 0;
        input_fd.events = POLLIN;
        const int poll_result = poll(&input_fd, 1, 0);
        if (poll_result > 0 && (input_fd.revents & (POLLIN | POLLHUP))) {
            std::string line;
            if (!std::getline(std::cin, line)) {
                exit_requested = true;
            } else {
                std::istringstream input(line);
                std::string command;
                input >> command;
                if (command == "speed") {
                    double requested = 0.0;
                    std::string extra;
                    if (!(input >> requested) || (input >> extra) ||
                        !std::isfinite(requested) || requested < 0.1 ||
                        requested > kCspLiveRatedRpm) {
                        std::cout << "Usage: speed <0.1 to "
                                  << kCspLiveRatedRpm << " rpm>\n";
                    } else {
                        speed_rpm = requested;
                        std::cout << "live_speed=" << speed_rpm << " rpm\n";
                    }
                } else if (command == "move_deg" || command == "turns") {
                    double value = 0.0;
                    std::string extra;
                    if (!(input >> value) || (input >> extra) ||
                        !std::isfinite(value)) {
                        std::cout << "Usage: " << command << " <nonzero-value>\n";
                    } else {
                        const double turns = command == "turns" ? value : value / 360.0;
                        const double delta_double = turns * kCountsPerRevolution;
                        const int64_t delta = static_cast<int64_t>(std::llround(delta_double));
                        const int64_t candidate = static_cast<int64_t>(target) + delta;
                        if (delta == 0 || std::abs(turns) > kCspLiveMaxRelativeTurns ||
                            candidate < std::numeric_limits<int32_t>::min() ||
                            candidate > std::numeric_limits<int32_t>::max()) {
                            std::cout << "Rejected: one relative target must be nonzero, within +/-"
                                      << kCspLiveMaxRelativeTurns
                                      << " turns, and must not overflow position.\n";
                        } else if (ecatm_arm_csp_motion() < 0) {
                            std::cerr << "Failed to arm the CSP trigger edge; stopping immediately.\n";
                            ok = false;
                        } else {
                            target = static_cast<int32_t>(candidate);
                            moving = true;
                            trigger_pending = true;
                            std::cout << "live_target=" << target
                                      << " count relative=" << counts_to_degrees(delta)
                                      << " deg speed=" << speed_rpm << " rpm\n";
                        }
                    }
                } else if (command == "hold") {
                    target = applied;
                    trajectory = static_cast<double>(applied);
                    moving = false;
                    ecatm_control(&applied);
                    std::cout << "Holding current trajectory position=" << applied << " count.\n";
                } else if (command == "status") {
                    Snapshot s;
                    if (read_snapshot(s)) {
                        std::cout << "live_origin=" << origin
                                  << " travel="
                                  << static_cast<int64_t>(s.position) - origin
                                  << " travel_deg="
                                  << counts_to_degrees(
                                         static_cast<int64_t>(s.position) - origin)
                                  << " target=" << target
                                  << " applied=" << applied
                                  << " remaining="
                                  << static_cast<int64_t>(target) - s.position
                                  << " speed_rpm=" << speed_rpm
                                  << " moving=" << (moving ? "YES" : "NO") << '\n';
                    }
                    print_snapshot();
                } else if (command == "stop" || command == "quit" ||
                           command == "exit") {
                    exit_requested = true;
                    std::cout << "Holding current position and disabling the drive.\n";
                } else if (command == "help") {
                    std::cout
                        << "speed <rpm> sets trajectory speed; move_deg <deg> adds a relative angle; "
                           "turns <turns> adds relative turns; hold stops and holds; "
                           "status prints state; stop disables and exits.\n";
                } else if (!command.empty()) {
                    std::cout << "Unknown live command; type help.\n";
                }
                if (!exit_requested && ok)
                    std::cout << "csp-live> " << std::flush;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    safe_command_and_disable();
    print_snapshot();
    return ok && !stop_requested ? 0 : 5;
}

int run_cyclic_raw(const std::string& mode_name, int32_t raw) {
    const bool csv = mode_name == "csv";
    const int32_t raw_limit = csv ? kCsvRawLimit : kCstRawLimit;
    const int32_t travel_limit = csv ? kCsvTravelLimit : kCstTravelLimit;
    if (raw == 0 || std::llabs(static_cast<long long>(raw)) > raw_limit) {
        std::cout << mode_name << " raw range is +/-" << raw_limit
                  << " and cannot be zero.\n";
        return 2;
    }
    if (!clear_fault_if_needed())
        return 3;
    std::ostringstream description;
    description << (csv ? "CSV velocity" : "CST torque")
                << " short pulse, raw=" << raw
                << "; this project has not confirmed the physical unit.";
    if (!confirm("ENABLE", description.str()))
        return 0;

    int32_t zero = 0;
    ecatm_control(&zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        std::cerr << mode_name << " enable failed.\n";
        return 4;
    }
    Snapshot enabled;
    read_snapshot(enabled);
    const int32_t origin = enabled.position;
    /* The original IPE project drives controlword bit 4 high in every cyclic
     * synchronous mode. Create a deliberate low-to-high edge and keep the
     * run word refreshed during the pulse. */
    write_controlword(CONTROLWORD_COMMAND_ENABLEOPERATION);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    bool ok = true;
    int cycle = 0;
    constexpr int ramp_ms = 200;
    constexpr int hold_ms = 300;
    constexpr int total_ms = ramp_ms + hold_ms + ramp_ms;
    for (int elapsed = 0; elapsed <= total_ms && !stop_requested && ok; ++elapsed) {
        int32_t command = 0;
        if (elapsed < ramp_ms) {
            command = static_cast<int32_t>(std::llround(
                static_cast<double>(raw) * elapsed / ramp_ms));
        } else if (elapsed < ramp_ms + hold_ms) {
            command = raw;
        } else {
            command = static_cast<int32_t>(std::llround(
                static_cast<double>(raw) * (total_ms - elapsed) / ramp_ms));
        }
        ecatm_control(&command);
        write_controlword(0x001f);
        ok = feedback_safe(origin, travel_limit, cycle++);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    write_controlword(CONTROLWORD_COMMAND_ENABLEOPERATION);
    safe_command_and_disable();
    print_snapshot();
    return ok && !stop_requested ? 0 : 5;
}

int run_csv_live() {
    if (!confirm(
            "NO_CABLE",
            "Continuous rotation requires a clear center bore; confirm no cable can be wound."))
        return 0;
    if (!clear_fault_if_needed())
        return 3;
    if (!confirm(
            "ENABLE",
            "CSV live control stays enabled. The measured 101:1 rated limit is +/-13107200 raw "
            "(3000 motor rpm, about 29.7 flange rpm). Use stop to exit."))
        return 0;

    int32_t zero = 0;
    ecatm_control(&zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        std::cerr << "CSV live velocity enable failed.\n";
        return 4;
    }

    Snapshot enabled;
    if (!read_snapshot(enabled)) {
        safe_command_and_disable();
        return 5;
    }
    const int32_t origin = enabled.position;
    write_controlword(CONTROLWORD_COMMAND_ENABLEOPERATION);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    int32_t desired = 0;
    int32_t applied = 0;
    int cycle = 0;
    bool ok = true;
    bool exit_after_zero = false;
    std::cout
        << "CSV live control started with zero velocity.\n"
        << "Commands: set <raw>, add <raw>, status, stop. Range +/-"
        << kCsvLiveRawLimit << " raw。\n"
        << "csv-live> " << std::flush;

    while (!stop_requested && ok) {
        if (applied < desired)
            applied = std::min<int32_t>(desired, applied + kCsvLiveSlewPerMs);
        else if (applied > desired)
            applied = std::max<int32_t>(desired, applied - kCsvLiveSlewPerMs);

        ecatm_control(&applied);
        write_controlword(0x001f);
        /* Keep all safety checks active without continuously redrawing the
         * terminal. The operator can request telemetry with `status`. */
        ok = feedback_safe(origin, kCsvLiveTravelLimit, cycle, 0);
        ++cycle;

        if (exit_after_zero && applied == 0)
            break;

        pollfd input_fd{};
        input_fd.fd = 0;
        input_fd.events = POLLIN;
        const int poll_result = poll(&input_fd, 1, 0);
        if (poll_result > 0 && (input_fd.revents & (POLLIN | POLLHUP))) {
            std::string line;
            if (!std::getline(std::cin, line)) {
                desired = 0;
                exit_after_zero = true;
            } else {
                std::istringstream input(line);
                std::string command;
                input >> command;
                if (command == "set" || command == "add") {
                    long long value = 0;
                    std::string extra;
                    if (!(input >> value) || (input >> extra)) {
                        std::cout << "Usage: " << command << " <integer>\n";
                    } else {
                        const long long lower_delta =
                            -static_cast<long long>(kCsvLiveRawLimit) - desired;
                        const long long upper_delta =
                            static_cast<long long>(kCsvLiveRawLimit) - desired;
                        const bool in_range = command == "set"
                            ? value >= -kCsvLiveRawLimit &&
                                  value <= kCsvLiveRawLimit
                            : value >= lower_delta && value <= upper_delta;
                        if (!in_range) {
                            std::cout << "Rejected: velocity must be within +/-" << kCsvLiveRawLimit
                                      << " raw.\n";
                        } else {
                            const long long candidate = command == "set"
                                ? value
                                : static_cast<long long>(desired) + value;
                            desired = static_cast<int32_t>(candidate);
                            std::cout << "live_target=" << desired << " raw\n";
                        }
                    }
                } else if (command == "status") {
                    Snapshot s;
                    if (read_snapshot(s)) {
                        const int64_t travel =
                            static_cast<int64_t>(s.position) - origin;
                        std::cout << "live_origin=" << origin
                                  << " travel=" << travel
                                  << " travel_deg=" << std::fixed
                                  << std::setprecision(4)
                                  << counts_to_degrees(travel)
                                  << " desired=" << desired
                                  << " applied=" << applied << '\n';
                    }
                    print_snapshot();
                } else if (command == "stop" || command == "quit" ||
                           command == "exit") {
                    desired = 0;
                    exit_after_zero = true;
                    std::cout << "Ramping velocity to zero and disabling.\n";
                } else if (command == "help") {
                    std::cout << "set <raw> sets velocity; add <raw> changes velocity; "
                                 "status prints state; stop disables and exits.\n";
                } else if (!command.empty()) {
                    std::cout << "Unknown live command; type help.\n";
                }
                if (!exit_after_zero)
                    std::cout << "csv-live> " << std::flush;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    zero = 0;
    ecatm_control(&zero);
    write_controlword(CONTROLWORD_COMMAND_ENABLEOPERATION);
    safe_command_and_disable();
    print_snapshot();
    return ok && !stop_requested ? 0 : 5;
}

int run_cst_live() {
    if (!confirm(
            "NO_CABLE",
            "Continuous CST can accelerate an unloaded joint; confirm no cable can be wound."))
        return 0;
    if (!clear_fault_if_needed())
        return 3;
    if (!confirm(
            "CST_LIVE",
            "CST physical units are unconfirmed. The unloaded limit is +/-300 raw, values above "
            "+/-100 raw warn, and velocity above +/-13107200 raw stops automatically."))
        return 0;
    if (!confirm("ENABLE", "CST live torque stays enabled; use stop to exit normally."))
        return 0;

    int32_t zero = 0;
    ecatm_control(&zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        std::cerr << "CST live torque enable failed.\n";
        return 4;
    }

    Snapshot enabled;
    if (!read_snapshot(enabled)) {
        safe_command_and_disable();
        return 5;
    }
    const int32_t origin = enabled.position;
    write_controlword(CONTROLWORD_COMMAND_ENABLEOPERATION);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    int32_t desired = 0;
    int32_t applied = 0;
    int cycle = 0;
    bool ok = true;
    bool exit_after_zero = false;
    std::cout
        << "CST live control started with zero torque.\n"
        << "Commands: set <raw>, add <raw>, status, stop. Range +/-"
        << kCstLiveRawLimit << " raw。\n"
        << "cst-live> " << std::flush;

    while (!stop_requested && ok) {
        /* Change by one raw unit every 20 ms: a zero-to-100 transition takes
         * two seconds instead of applying an abrupt unknown physical torque. */
        if (cycle % 20 == 0) {
            if (applied < desired)
                ++applied;
            else if (applied > desired)
                --applied;
        }

        ecatm_control(&applied);
        write_controlword(0x001f);
        ok = feedback_safe(origin, 0, cycle, 0, kCstLiveVelocityLimit);
        ++cycle;

        if (exit_after_zero && applied == 0)
            break;

        pollfd input_fd{};
        input_fd.fd = 0;
        input_fd.events = POLLIN;
        const int poll_result = poll(&input_fd, 1, 0);
        if (poll_result > 0 && (input_fd.revents & (POLLIN | POLLHUP))) {
            std::string line;
            if (!std::getline(std::cin, line)) {
                desired = 0;
                exit_after_zero = true;
            } else {
                std::istringstream input(line);
                std::string command;
                input >> command;
                if (command == "set" || command == "add") {
                    long long value = 0;
                    std::string extra;
                    if (!(input >> value) || (input >> extra)) {
                        std::cout << "Usage: " << command << " <integer>\n";
                    } else {
                        const long long lower_delta =
                            -static_cast<long long>(kCstLiveRawLimit) - desired;
                        const long long upper_delta =
                            static_cast<long long>(kCstLiveRawLimit) - desired;
                        const bool in_range = command == "set"
                            ? value >= -kCstLiveRawLimit &&
                                  value <= kCstLiveRawLimit
                            : value >= lower_delta && value <= upper_delta;
                        if (!in_range) {
                            std::cout << "Rejected: torque must be within +/-" << kCstLiveRawLimit
                                      << " raw.\n";
                        } else {
                            desired = static_cast<int32_t>(command == "set"
                                ? value
                                : static_cast<long long>(desired) + value);
                            std::cout << "live_target=" << desired << " raw\n";
                            if (std::llabs(static_cast<long long>(desired)) >
                                kCstLiveCautionBoundary) {
                                std::cout
                                    << "Warning: command exceeds the completed unloaded test range of +/-"
                                    << kCstLiveCautionBoundary
                                    << " raw; continuously monitor speed, current, and temperature.\n";
                            }
                        }
                    }
                } else if (command == "status") {
                    Snapshot s;
                    if (read_snapshot(s)) {
                        const int64_t travel =
                            static_cast<int64_t>(s.position) - origin;
                        std::cout << "live_origin=" << origin
                                  << " travel=" << travel
                                  << " travel_deg=" << std::fixed
                                  << std::setprecision(4)
                                  << counts_to_degrees(travel)
                                  << " desired=" << desired
                                  << " applied=" << applied << '\n';
                    }
                    print_snapshot();
                } else if (command == "stop" || command == "quit" ||
                           command == "exit") {
                    desired = 0;
                    exit_after_zero = true;
                    std::cout << "Ramping torque to zero and disabling.\n";
                } else if (command == "help") {
                    std::cout << "set <raw> sets torque; add <raw> changes torque; "
                                 "status prints state; stop disables and exits.\n";
                } else if (!command.empty()) {
                    std::cout << "Unknown live command; type help.\n";
                }
                if (!exit_after_zero)
                    std::cout << "cst-live> " << std::flush;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    zero = 0;
    ecatm_control(&zero);
    write_controlword(CONTROLWORD_COMMAND_ENABLEOPERATION);
    safe_command_and_disable();
    print_snapshot();
    return ok && !stop_requested ? 0 : 5;
}

void print_help() {
    std::cout
        << "Commands:\n"
        << "  status              Show all targets and actual feedback\n"
        << "  mode csp|csv|cst    Disable and switch mode within this connection\n"
        << "  reset               Explicit reset only when the drive is in Fault\n"
        << "  run <raw>           CSP: relative count and return; CSV/CST: short pulse\n"
        << "  live                CSP trajectory, CSV continuous velocity, or CST continuous torque\n"
        << "  help                Show help\n"
        << "  quit                Disable and close EtherCAT\n"
        << "Limits: CSP short test +/-1000 count and about 29.7 rpm live; "
           "CSV pulse +/-5000 raw; CSV live rated limit +/-13107200 raw (measured 101:1); "
           "CST pulse +/-10 raw and unloaded live limit +/-300 raw.\n";
}

int console() {
    std::string active_mode = "csp";
    print_help();
    print_snapshot();
    std::string line;
    while (!stop_requested) {
        std::cout << "ipe[" << active_mode << "]> " << std::flush;
        if (!std::getline(std::cin, line))
            break;
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command.empty())
            continue;
        if (command == "status") {
            print_snapshot();
        } else if (command == "help") {
            print_help();
        } else if (command == "reset") {
            if (!clear_fault_if_needed())
                std::cout << "Reset did not complete.\n";
        } else if (command == "mode") {
            std::string requested;
            std::string extra;
            if (!(input >> requested) || (input >> extra) ||
                expected_mode(requested) == 0) {
                std::cout << "Usage: mode csp|csv|cst\n";
                continue;
            }
            if (select_mode(requested))
                active_mode = requested;
            print_snapshot();
        } else if (command == "run") {
            long long raw = 0;
            std::string extra;
            if (!(input >> raw) || (input >> extra) ||
                raw < std::numeric_limits<int32_t>::min() ||
                raw > std::numeric_limits<int32_t>::max()) {
                std::cout << "Usage: run <integer>\n";
                continue;
            }
            const int result = active_mode == "csp"
                ? run_csp(static_cast<int32_t>(raw))
                : run_cyclic_raw(active_mode, static_cast<int32_t>(raw));
            std::cout << "run_result=" << (result == 0 ? "OK" : "NOT_OK")
                      << " code=" << result << '\n';
        } else if (command == "live") {
            std::string extra;
            if (input >> extra) {
                std::cout << "Usage: live\n";
                continue;
            }
            const int result = active_mode == "csp"
                ? run_csp_live()
                : active_mode == "csv" ? run_csv_live() : run_cst_live();
            std::cout << "live_result=" << (result == 0 ? "OK" : "NOT_OK")
                      << " code=" << result << '\n';
        } else if (command == "quit" || command == "exit") {
            break;
        } else {
            std::cout << "Unknown command; type help.\n";
        }
    }
    ecatm_disable();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    const std::string interface = argc >= 2 ? argv[1] : "enp130s0";
    if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [interface]\n";
        return 2;
    }
    if (ecatm_set_interface(interface.c_str()) < 0) {
        std::cerr << "Network interface is unavailable: " << interface << '\n';
        return 2;
    }
    int result = ecatm_init_passive("csp", 1);
    if (result < 0) {
        std::cerr << "EtherCAT initialization failed: " << result << '\n';
        return 1;
    }
    uint32_t vendor = 0;
    uint32_t product = 0;
    uint32_t revision = 0;
    if (ecatm_get_slave_identity(0, &vendor, &product, &revision) < 0 ||
        vendor != kVendor || product != kProduct) {
        std::cerr << "Slave identity mismatch; enable is prohibited. vendor=0x" << std::hex
                  << vendor << " product=0x" << product << std::dec << '\n';
        ecatm_stop();
        return 1;
    }
    std::cout << "IPE identity verified: vendor=0x" << std::hex << vendor
              << " product=0x" << product << " revision=0x" << revision
              << std::dec << "; passive CSP is selected and the drive is disabled.\n";
    result = console();
    std::cout << "Closing EtherCAT and returning to SAFE-OP.\n";
    ecatm_stop();
    return result;
}
