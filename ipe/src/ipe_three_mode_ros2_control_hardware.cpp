#include "ipe_three_mode_ros2_control_hardware.hpp"

#include "cia402_def.h"
#include "ecat_motor_master.h"
#include "ipe_joint_units.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <system_error>
#include <thread>

namespace ipe_ros2_control {
namespace {

constexpr uint32_t kIpeVendorId = 0x00041101;
constexpr uint32_t kIrgmlProductId = 0x00009253;
constexpr const char* kMotionAuthorization = "ENABLE_THREE_MODE_MOTION";

template <typename T>
bool parse_integer(const std::string& text, T* value) {
    if (!value)
        return false;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), *value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

bool has(const std::vector<std::string>& values, const std::string& expected) {
    return std::find(values.begin(), values.end(), expected) != values.end();
}

int32_t slew_toward(int32_t current, int32_t target, int32_t max_step) {
    const int64_t difference = static_cast<int64_t>(target) - current;
    if (std::llabs(difference) <= max_step)
        return target;
    return current + (difference > 0 ? max_step : -max_step);
}

bool finite_int32(double value, int32_t* result) {
    if (!result || !std::isfinite(value) ||
        value < std::numeric_limits<int32_t>::min() ||
        value > std::numeric_limits<int32_t>::max())
        return false;
    *result = static_cast<int32_t>(std::llround(value));
    return true;
}

bool parse_positive_double(const std::string& text, double* value) {
    if (!value)
        return false;
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (end != text.c_str() + text.size() || !std::isfinite(parsed) || parsed <= 0.0)
        return false;
    *value = parsed;
    return true;
}

}  // namespace

IpeThreeModeSystem::~IpeThreeModeSystem() { stop_master(); }

hardware_interface::CallbackReturn IpeThreeModeSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    const auto& info = get_hardware_info();
    if (info.joints.size() != 1) {
        RCLCPP_ERROR(get_logger(), "IPE three-mode hardware requires one joint");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto& joint = info.joints.front();
    const auto has_state = [&joint](const std::string& name) {
        return std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
                           [&name](const auto& item) { return item.name == name; });
    };
    const auto has_command = [&joint](const std::string& name) {
        return std::any_of(joint.command_interfaces.begin(), joint.command_interfaces.end(),
                           [&name](const auto& item) { return item.name == name; });
    };
    if (joint.state_interfaces.size() != 4 ||
        !has_state("position") || !has_state("velocity") || !has_state("velocity_raw") ||
        !has_state("torque_raw") || joint.command_interfaces.size() != 3 ||
        !has_command("position") || !has_command("velocity_raw") ||
        !has_command("torque_raw")) {
        RCLCPP_ERROR(get_logger(),
                     "IPE three-mode joint needs position, velocity, velocity_raw and torque_raw "
                     "state and command interfaces");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto parameter = [&info](const char* name) -> const std::string* {
        const auto it = info.hardware_parameters.find(name);
        return it == info.hardware_parameters.end() ? nullptr : &it->second;
    };
    const std::string* interface = parameter("interface");
    const std::string* zero = parameter("zero_count");
    const std::string* direction = parameter("direction");
    const std::string* raw_direction = parameter("raw_direction");
    const std::string* authorization = parameter("motion_authorization");
    const std::string* csp_travel = parameter("csp_max_travel_count");
    const std::string* csp_step = parameter("csp_max_step_count");
    const std::string* csp_error = parameter("csp_max_following_error_count");
    const std::string* csv_raw = parameter("csv_max_raw");
    const std::string* csv_step = parameter("csv_max_step_raw");
    const std::string* csv_travel = parameter("csv_max_travel_count");
    const std::string* cst_raw = parameter("cst_max_raw");
    const std::string* cst_step = parameter("cst_max_step_raw");
    const std::string* cst_travel = parameter("cst_max_travel_count");
    const std::string* cst_slew_interval =
        parameter("cst_slew_interval_cycles");
    const std::string* rated_velocity = parameter("rated_velocity_raw");
    const std::string* velocity_scale = parameter("velocity_raw_to_rad_s");
    if (!interface || !zero || !direction || !authorization || !csp_travel ||
        !csp_step || !csp_error || !csv_raw || !csv_step || !csv_travel ||
        !cst_raw || !cst_step || !cst_travel || !cst_slew_interval ||
        !rated_velocity || !velocity_scale ||
        *authorization != kMotionAuthorization) {
        RCLCPP_ERROR(get_logger(), "Missing or invalid three-mode safety parameters");
        return hardware_interface::CallbackReturn::ERROR;
    }

    interface_name_ = *interface;
    joint_name_ = joint.name;
    position_state_interface_name_ = joint_name_ + "/position";
    velocity_si_state_interface_name_ = joint_name_ + "/velocity";
    velocity_state_interface_name_ = joint_name_ + "/velocity_raw";
    torque_state_interface_name_ = joint_name_ + "/torque_raw";
    position_command_interface_name_ = position_state_interface_name_;
    velocity_command_interface_name_ = velocity_state_interface_name_;
    torque_command_interface_name_ = torque_state_interface_name_;
    if (interface_name_.empty() || !parse_integer(*zero, &zero_count_) ||
        !parse_integer(*direction, &direction_) ||
        (raw_direction && !parse_integer(*raw_direction, &raw_direction_)) ||
        !parse_integer(*csp_travel, &csp_max_travel_count_) ||
        !parse_integer(*csp_step, &csp_max_step_count_) ||
        !parse_integer(*csp_error, &csp_max_following_error_count_) ||
        !parse_integer(*csv_raw, &csv_max_raw_) ||
        !parse_integer(*csv_step, &csv_max_step_raw_) ||
        !parse_integer(*csv_travel, &csv_max_travel_count_) ||
        !parse_integer(*cst_raw, &cst_max_raw_) ||
        !parse_integer(*cst_step, &cst_max_step_raw_) ||
        !parse_integer(*cst_travel, &cst_max_travel_count_) ||
        !parse_integer(*cst_slew_interval, &cst_slew_interval_cycles_) ||
        !parse_integer(*rated_velocity, &rated_velocity_raw_) ||
        !parse_positive_double(*velocity_scale, &velocity_raw_to_rad_s_) ||
        !ipe::joint_units::is_valid_direction(direction_) ||
        !ipe::joint_units::is_valid_direction(raw_direction_) ||
        csp_max_travel_count_ <= 0 || csp_max_travel_count_ > 26214400 ||
        csp_max_step_count_ <= 0 || csp_max_step_count_ > 1200 ||
        csp_max_following_error_count_ <= 0 ||
        csp_max_following_error_count_ > csp_max_travel_count_ ||
        csv_max_raw_ <= 0 || csv_max_raw_ > 13107200 ||
        csv_max_step_raw_ <= 0 || csv_max_step_raw_ > 10000 ||
        csv_max_travel_count_ < 0 || csv_max_travel_count_ > 1000 ||
        cst_max_raw_ <= 0 || cst_max_raw_ > 300 ||
        cst_max_step_raw_ <= 0 || cst_max_step_raw_ > cst_max_raw_ ||
        cst_max_travel_count_ < 0 || cst_max_travel_count_ > 500 ||
        cst_slew_interval_cycles_ <= 0 || cst_slew_interval_cycles_ > 100 ||
        rated_velocity_raw_ <= 0 || rated_velocity_raw_ > 13107200) {
        RCLCPP_ERROR(get_logger(), "Unsafe or invalid three-mode parameter value");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(),
                "Initialized IPE three-mode metadata: CSP +/- %d count "
                "(step=%d), CSV +/- %d raw, CST +/- %d raw, "
                "rated velocity +/- %d raw",
                csp_max_travel_count_, csp_max_step_count_, csv_max_raw_,
                cst_max_raw_, rated_velocity_raw_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeThreeModeSystem::on_configure(
    const rclcpp_lifecycle::State&) {
    if (ecatm_set_interface(interface_name_.c_str()) < 0 ||
        ecatm_init_passive("csp", 1) < 0) {
        RCLCPP_ERROR(get_logger(), "Passive EtherCAT initialization failed");
        stop_master();
        return hardware_interface::CallbackReturn::ERROR;
    }
    master_started_ = true;
    uint32_t vendor = 0, product = 0, revision = 0;
    if (ecatm_get_slave_identity(0, &vendor, &product, &revision) < 0 ||
        vendor != kIpeVendorId || product != kIrgmlProductId) {
        RCLCPP_ERROR(get_logger(),
                     "Unexpected slave: vendor=0x%08x product=0x%08x revision=0x%08x",
                     vendor, product, revision);
        stop_master();
        return hardware_interface::CallbackReturn::ERROR;
    }
    esc_get_states(&actual_count_, &actual_velocity_, &actual_torque_);
    set_state(position_state_interface_name_, ipe::joint_units::position_to_radians(
        actual_count_, zero_count_, direction_));
    set_state(velocity_si_state_interface_name_,
              static_cast<double>(actual_velocity_) * velocity_raw_to_rad_s_ * direction_);
    set_state(velocity_state_interface_name_,
              static_cast<double>(actual_velocity_) * raw_direction_);
    set_state(torque_state_interface_name_,
              static_cast<double>(actual_torque_) * raw_direction_);
    set_command(position_command_interface_name_, get_state<double>(position_state_interface_name_));
    set_command(velocity_command_interface_name_, 0.0);
    set_command(torque_command_interface_name_, 0.0);
    RCLCPP_INFO(get_logger(),
                "Three-mode EtherCAT configured at count=%d; drive remains disabled",
                actual_count_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeThreeModeSystem::on_activate(
    const rclcpp_lifecycle::State&) {
    hardware_active_ = true;
    RCLCPP_WARN(get_logger(),
                "Three-mode hardware active with drive disabled; activating exactly "
                "one motion controller enables its mode");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeThreeModeSystem::on_deactivate(
    const rclcpp_lifecycle::State&) {
    disable_commanding();
    hardware_active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeThreeModeSystem::on_cleanup(
    const rclcpp_lifecycle::State&) { stop_master(); return hardware_interface::CallbackReturn::SUCCESS; }
hardware_interface::CallbackReturn IpeThreeModeSystem::on_shutdown(
    const rclcpp_lifecycle::State&) { stop_master(); return hardware_interface::CallbackReturn::SUCCESS; }
hardware_interface::CallbackReturn IpeThreeModeSystem::on_error(
    const rclcpp_lifecycle::State&) { stop_master(); return hardware_interface::CallbackReturn::SUCCESS; }

IpeThreeModeSystem::Mode IpeThreeModeSystem::requested_mode(
    const std::vector<std::string>& interfaces, bool* multiple) const {
    int count = 0;
    Mode mode = Mode::None;
    if (has(interfaces, position_command_interface_name_)) { mode = Mode::Csp; ++count; }
    if (has(interfaces, velocity_command_interface_name_)) { mode = Mode::Csv; ++count; }
    if (has(interfaces, torque_command_interface_name_)) { mode = Mode::Cst; ++count; }
    if (multiple) *multiple = count > 1;
    return mode;
}

hardware_interface::return_type IpeThreeModeSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
    bool multiple = false;
    const Mode start = requested_mode(start_interfaces, &multiple);
    if (multiple) {
        RCLCPP_ERROR(get_logger(), "Only one of CSP, CSV and CST may be active");
        return hardware_interface::return_type::ERROR;
    }
    const Mode stop = requested_mode(stop_interfaces, &multiple);
    if (multiple) {
        RCLCPP_ERROR(get_logger(), "Invalid multi-mode stop request");
        return hardware_interface::return_type::ERROR;
    }
    pending_mode_ = Mode::None;
    pending_stop_ = false;
    if (stop != Mode::None || (start != Mode::None && commanding_.load())) {
        disable_commanding();
        pending_stop_ = true;
    }
    if (start != Mode::None) {
        if (!enable_mode(start))
            return hardware_interface::return_type::ERROR;
        pending_mode_ = start;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IpeThreeModeSystem::perform_command_mode_switch(
    const std::vector<std::string>&, const std::vector<std::string>&) {
    if (pending_stop_ && pending_mode_ == Mode::None) {
        active_mode_.store(Mode::None);
        commanding_.store(false);
    }
    if (pending_mode_ != Mode::None) {
        active_mode_.store(pending_mode_);
        commanding_.store(true);
    }
    pending_mode_ = Mode::None;
    pending_stop_ = false;
    return hardware_interface::return_type::OK;
}

bool IpeThreeModeSystem::enable_mode(Mode mode) {
    if (!master_started_ || !hardware_active_ || !ecatm_is_pdo_healthy()) {
        RCLCPP_ERROR(get_logger(), "Cannot enable mode: EtherCAT/PDO is not healthy");
        return false;
    }
    if (ecatm_switch_mode(mode_name(mode)) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to select %s", mode_name(mode));
        return false;
    }
    uint16_t status = 0;
    uint32_t error = 0;
    esc_get_status_word(&status);
    esc_get_error_codes(&error);
    if ((status & STATUSWORD_STATE_MASK) == STATUSWORD_STATE_FAULT &&
        ecatm_fault_reset() < 0) {
        RCLCPP_ERROR(get_logger(), "Fault reset failed: error=0x%08x", error);
        return false;
    }

    esc_get_states(&actual_count_, &actual_velocity_, &actual_torque_);
    activation_origin_count_ = actual_count_;
    applied_command_ = mode == Mode::Csp ? actual_count_ : 0;
    cst_slew_cycle_ = 0;
    set_command(position_command_interface_name_, ipe::joint_units::position_to_radians(
        actual_count_, zero_count_, direction_));
    set_command(velocity_command_interface_name_, 0.0);
    set_command(torque_command_interface_name_, 0.0);
    ecatm_control(&applied_command_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        RCLCPP_ERROR(get_logger(), "%s drive enable failed", mode_name(mode));
        return false;
    }
    active_mode_.store(mode);
    drive_enabled_.store(true);
    motion_active_ = false;
    RCLCPP_WARN(get_logger(), "%s enabled at origin=%d count", mode_name(mode),
                activation_origin_count_);
    return true;
}

void IpeThreeModeSystem::disable_commanding() {
    commanding_.store(false);
    motion_active_ = false;
    const bool was_enabled = drive_enabled_.exchange(false);
    if (was_enabled) {
        int32_t safe = active_mode_.load() == Mode::Csp ? actual_count_ : 0;
        ecatm_control(&safe);
        ecatm_disable();
        RCLCPP_INFO(get_logger(), "%s drive disabled", mode_name(active_mode_.load()));
    }
}

hardware_interface::return_type IpeThreeModeSystem::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!master_started_ || !ecatm_is_pdo_healthy()) {
        RCLCPP_ERROR(get_logger(), "Three-mode PDO unhealthy; disabling motion");
        disable_commanding();
        return hardware_interface::return_type::ERROR;
    }
    esc_get_states(&actual_count_, &actual_velocity_, &actual_torque_);
    set_state(position_state_interface_name_, ipe::joint_units::position_to_radians(
        actual_count_, zero_count_, direction_));
    set_state(velocity_si_state_interface_name_,
              static_cast<double>(actual_velocity_) * velocity_raw_to_rad_s_ * direction_);
    set_state(velocity_state_interface_name_,
              static_cast<double>(actual_velocity_) * raw_direction_);
    set_state(torque_state_interface_name_,
              static_cast<double>(actual_torque_) * raw_direction_);
    if (drive_enabled_.load()) {
        uint16_t status = 0;
        uint32_t error = 0;
        esc_get_status_word(&status);
        esc_get_error_codes(&error);
        if ((status & STATUSWORD_STATE_MASK) != STATUSWORD_STATE_OPERATIONENABLED ||
            error != 0) {
            RCLCPP_ERROR(get_logger(),
                         "Drive left Operation enabled: status=0x%04x error=0x%08x",
                         status, error);
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        const Mode mode = active_mode_.load();
        if (std::llabs(static_cast<int64_t>(actual_velocity_)) >
            rated_velocity_raw_) {
            RCLCPP_ERROR(get_logger(),
                         "%s velocity=%d raw exceeds rated +/- %d raw",
                         mode_name(mode), actual_velocity_, rated_velocity_raw_);
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        const int64_t travel = static_cast<int64_t>(actual_count_) - activation_origin_count_;
        const int32_t travel_limit = mode == Mode::Csv ? csv_max_travel_count_ :
                                     mode == Mode::Cst ? cst_max_travel_count_ : 0;
        if (travel_limit > 0 && std::llabs(travel) > travel_limit) {
            RCLCPP_ERROR(get_logger(), "%s travel=%ld exceeds +/- %d count",
                         mode_name(mode), static_cast<long>(travel), travel_limit);
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IpeThreeModeSystem::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!commanding_.load() || !drive_enabled_.load())
        return hardware_interface::return_type::OK;
    const Mode mode = active_mode_.load();
    int32_t requested = 0;
    if (mode == Mode::Csp) {
        if (!ipe::joint_units::radians_to_position(
                get_command<double>(position_command_interface_name_), zero_count_,
                direction_, &requested)) {
            RCLCPP_ERROR(get_logger(), "Rejected invalid CSP position command");
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        const int64_t travel = static_cast<int64_t>(requested) - activation_origin_count_;
        const int64_t following = static_cast<int64_t>(applied_command_) - actual_count_;
        if (std::llabs(travel) > csp_max_travel_count_ ||
            std::llabs(following) > csp_max_following_error_count_) {
            RCLCPP_ERROR(get_logger(), "Rejected CSP travel/following error");
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        requested = slew_toward(applied_command_, requested, csp_max_step_count_);
    } else if (mode == Mode::Csv) {
        if (!finite_int32(get_command<double>(velocity_command_interface_name_), &requested) ||
            std::llabs(static_cast<int64_t>(requested)) > csv_max_raw_) {
            RCLCPP_ERROR(get_logger(), "Rejected CSV raw command outside +/- %d", csv_max_raw_);
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        requested *= raw_direction_;
        requested = slew_toward(applied_command_, requested, csv_max_step_raw_);
    } else if (mode == Mode::Cst) {
        if (!finite_int32(get_command<double>(torque_command_interface_name_), &requested) ||
            std::llabs(static_cast<int64_t>(requested)) > cst_max_raw_) {
            RCLCPP_ERROR(get_logger(), "Rejected CST raw command outside +/- %d", cst_max_raw_);
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        requested *= raw_direction_;
        if (++cst_slew_cycle_ >= cst_slew_interval_cycles_) {
            requested = slew_toward(
                applied_command_, requested, cst_max_step_raw_);
            cst_slew_cycle_ = 0;
        } else {
            requested = applied_command_;
        }
    } else {
        return hardware_interface::return_type::OK;
    }

    const bool should_move = mode == Mode::Csp ? requested != applied_command_ : requested != 0;
    if (should_move && !motion_active_) {
        if (ecatm_arm_cyclic_motion() < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to arm %s motion edge", mode_name(mode));
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        motion_active_ = true;
    }
    applied_command_ = requested;
    ecatm_control(&applied_command_);
    if (should_move && motion_active_ && ecatm_commit_cyclic_motion() < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to commit %s motion edge", mode_name(mode));
        disable_commanding();
        return hardware_interface::return_type::ERROR;
    }
    if (mode == Mode::Csp && !should_move &&
        std::llabs(static_cast<int64_t>(applied_command_) - actual_count_) <= 8)
        motion_active_ = false;
    if (mode != Mode::Csp && applied_command_ == 0)
        motion_active_ = false;
    return hardware_interface::return_type::OK;
}

const char* IpeThreeModeSystem::mode_name(Mode mode) const {
    switch (mode) {
        case Mode::Csp: return "csp";
        case Mode::Csv: return "csv";
        case Mode::Cst: return "cst";
        default: return "none";
    }
}

void IpeThreeModeSystem::stop_master() {
    disable_commanding();
    if (master_started_) {
        ecatm_stop();
        master_started_ = false;
    }
    active_mode_.store(Mode::None);
    hardware_active_ = false;
}

}  // namespace ipe_ros2_control

PLUGINLIB_EXPORT_CLASS(ipe_ros2_control::IpeThreeModeSystem,
                       hardware_interface::SystemInterface)
