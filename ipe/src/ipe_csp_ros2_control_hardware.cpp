#include "ipe_csp_ros2_control_hardware.hpp"

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
constexpr const char* kMotionAuthorization = "ENABLE_CSP_MOTION";
constexpr int kTargetStableCycles = 10;
constexpr int32_t kTargetToleranceCount = 8;

template <typename T>
bool parse_integer(const std::string& text, T* value) {
    if (!value)
        return false;
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto result = std::from_chars(begin, end, *value);
    return result.ec == std::errc{} && result.ptr == end;
}

bool contains(const std::vector<std::string>& values,
              const std::string& expected) {
    return std::find(values.begin(), values.end(), expected) != values.end();
}

int32_t slew_toward(int32_t current, int32_t target, int32_t max_step) {
    const int64_t difference = static_cast<int64_t>(target) - current;
    if (std::llabs(difference) <= max_step)
        return target;
    return static_cast<int32_t>(current + (difference > 0 ? max_step : -max_step));
}

}  // namespace

IpeCspSystem::~IpeCspSystem() {
    stop_master();
}

hardware_interface::CallbackReturn IpeCspSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto& info = get_hardware_info();
    if (info.joints.size() != 1 ||
        info.joints.front().state_interfaces.size() != 1 ||
        info.joints.front().state_interfaces.front().name !=
            hardware_interface::HW_IF_POSITION ||
        info.joints.front().command_interfaces.size() != 1 ||
        info.joints.front().command_interfaces.front().name !=
            hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(get_logger(),
                     "IPE CSP hardware requires exactly one joint with one "
                     "position state and one position command interface");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto parameter = [&info](const char* name) -> const std::string* {
        const auto it = info.hardware_parameters.find(name);
        return it == info.hardware_parameters.end() ? nullptr : &it->second;
    };
    const std::string* interface = parameter("interface");
    const std::string* zero = parameter("zero_count");
    const std::string* direction = parameter("direction");
    const std::string* authorization = parameter("motion_authorization");
    const std::string* max_travel = parameter("max_travel_count");
    const std::string* max_step = parameter("max_step_count");
    const std::string* max_error = parameter("max_following_error_count");
    if (!interface || !zero || !direction || !authorization || !max_travel ||
        !max_step || !max_error || *authorization != kMotionAuthorization) {
        RCLCPP_ERROR(get_logger(),
                     "Missing CSP safety parameters or motion_authorization is invalid");
        return hardware_interface::CallbackReturn::ERROR;
    }

    interface_name_ = *interface;
    joint_name_ = info.joints.front().name;
    position_state_interface_name_ =
        joint_name_ + "/" + hardware_interface::HW_IF_POSITION;
    position_command_interface_name_ = position_state_interface_name_;
    if (interface_name_.empty() || !parse_integer(*zero, &zero_count_) ||
        !parse_integer(*direction, &direction_) ||
        !parse_integer(*max_travel, &max_travel_count_) ||
        !parse_integer(*max_step, &max_step_count_) ||
        !parse_integer(*max_error, &max_following_error_count_) ||
        !ipe::joint_units::is_valid_direction(direction_) ||
        max_travel_count_ <= 0 || max_travel_count_ > 1000 ||
        max_step_count_ <= 0 || max_step_count_ > 20 ||
        max_following_error_count_ <= 0 ||
        max_following_error_count_ > max_travel_count_) {
        RCLCPP_ERROR(get_logger(), "Invalid IPE CSP safety parameter values");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(),
                "Initialized IPE CSP metadata: joint=%s interface=%s "
                "travel=+/- %d count step=%d count following_error=%d count",
                joint_name_.c_str(), interface_name_.c_str(), max_travel_count_,
                max_step_count_, max_following_error_count_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeCspSystem::on_configure(
    const rclcpp_lifecycle::State&) {
    if (ecatm_set_interface(interface_name_.c_str()) < 0 ||
        ecatm_init_passive("csp", 1) < 0) {
        RCLCPP_ERROR(get_logger(), "Passive CSP EtherCAT initialization failed");
        stop_master();
        return hardware_interface::CallbackReturn::ERROR;
    }
    master_started_ = true;

    uint32_t vendor = 0;
    uint32_t product = 0;
    uint32_t revision = 0;
    if (ecatm_get_slave_identity(0, &vendor, &product, &revision) < 0 ||
        vendor != kIpeVendorId || product != kIrgmlProductId) {
        RCLCPP_ERROR(get_logger(),
                     "Unexpected EtherCAT slave identity: vendor=0x%08x "
                     "product=0x%08x revision=0x%08x",
                     vendor, product, revision);
        stop_master();
        return hardware_interface::CallbackReturn::ERROR;
    }

    int32_t velocity = 0;
    int32_t torque = 0;
    esc_get_states(&actual_count_, &velocity, &torque);
    activation_origin_count_ = actual_count_;
    applied_command_count_ = actual_count_;
    const double position = ipe::joint_units::position_to_radians(
        actual_count_, zero_count_, direction_);
    set_state(position_state_interface_name_, position);
    set_command(position_command_interface_name_, position);
    RCLCPP_INFO(get_logger(),
                "Configured passive CSP EtherCAT at count=%d; drive remains disabled",
                actual_count_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeCspSystem::on_activate(
    const rclcpp_lifecycle::State&) {
    hardware_active_ = true;
    RCLCPP_WARN(get_logger(),
                "CSP hardware active but drive disabled; activating the position "
                "controller is the explicit motion-enable action");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeCspSystem::on_deactivate(
    const rclcpp_lifecycle::State&) {
    disable_commanding();
    hardware_active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeCspSystem::on_cleanup(
    const rclcpp_lifecycle::State&) {
    stop_master();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeCspSystem::on_shutdown(
    const rclcpp_lifecycle::State&) {
    stop_master();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeCspSystem::on_error(
    const rclcpp_lifecycle::State&) {
    RCLCPP_ERROR(get_logger(), "CSP hardware error; disabling and stopping EtherCAT");
    stop_master();
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool IpeCspSystem::command_interface_requested(
    const std::vector<std::string>& interfaces) const {
    return contains(interfaces, position_command_interface_name_);
}

hardware_interface::return_type IpeCspSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
    const bool start = command_interface_requested(start_interfaces);
    const bool stop = command_interface_requested(stop_interfaces);
    if (start && stop) {
        RCLCPP_ERROR(get_logger(), "Cannot start and stop CSP interface together");
        return hardware_interface::return_type::ERROR;
    }
    pending_start_ = false;
    pending_stop_ = false;
    if (start && !commanding_.load()) {
        if (!enable_commanding())
            return hardware_interface::return_type::ERROR;
        pending_start_ = true;
    } else if (stop && commanding_.load()) {
        disable_commanding();
        pending_stop_ = true;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IpeCspSystem::perform_command_mode_switch(
    const std::vector<std::string>&, const std::vector<std::string>&) {
    if (pending_stop_)
        commanding_.store(false);
    if (pending_start_)
        commanding_.store(true);
    pending_start_ = false;
    pending_stop_ = false;
    return hardware_interface::return_type::OK;
}

bool IpeCspSystem::enable_commanding() {
    if (!master_started_ || !hardware_active_ || !ecatm_is_pdo_healthy()) {
        RCLCPP_ERROR(get_logger(), "Cannot enable CSP: hardware or PDO is not healthy");
        return false;
    }

    uint16_t status = 0;
    uint32_t error = 0;
    esc_get_status_word(&status);
    esc_get_error_codes(&error);
    if ((status & STATUSWORD_STATE_MASK) == STATUSWORD_STATE_FAULT) {
        RCLCPP_WARN(get_logger(),
                    "Resetting latched drive fault before explicit CSP controller activation: "
                    "error=0x%08x",
                    error);
        if (ecatm_fault_reset() < 0) {
            RCLCPP_ERROR(get_logger(), "CSP fault reset failed");
            return false;
        }
    }

    int32_t velocity = 0;
    int32_t torque = 0;
    esc_get_states(&actual_count_, &velocity, &torque);
    activation_origin_count_ = actual_count_;
    applied_command_count_ = actual_count_;
    const double position = ipe::joint_units::position_to_radians(
        actual_count_, zero_count_, direction_);
    set_state(position_state_interface_name_, position);
    set_command(position_command_interface_name_, position);
    ecatm_control(&applied_command_count_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (ecatm_enable() < 0) {
        RCLCPP_ERROR(get_logger(), "CSP drive enable failed");
        return false;
    }

    drive_enabled_.store(true);
    motion_active_ = false;
    stable_cycles_ = 0;
    RCLCPP_WARN(get_logger(),
                "CSP drive enabled at origin=%d count; commands are limited to +/- %d count",
                activation_origin_count_, max_travel_count_);
    return true;
}

void IpeCspSystem::disable_commanding() {
    commanding_.store(false);
    pending_start_ = false;
    pending_stop_ = false;
    motion_active_ = false;
    stable_cycles_ = 0;
    // Publish the software-disabled state before the blocking CiA 402
    // transition. read() runs concurrently with controller switching and must
    // not interpret the expected Operation enabled -> Ready transition as a
    // drive fault.
    const bool was_enabled = drive_enabled_.exchange(false);
    if (was_enabled) {
        ecatm_control(&actual_count_);
        ecatm_disable();
        RCLCPP_INFO(get_logger(), "CSP drive disabled");
    }
}

hardware_interface::return_type IpeCspSystem::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!master_started_)
        return hardware_interface::return_type::ERROR;
    if (!ecatm_is_pdo_healthy()) {
        RCLCPP_ERROR(get_logger(), "CSP PDO unhealthy; disabling motion immediately");
        disable_commanding();
        return hardware_interface::return_type::ERROR;
    }

    int32_t velocity = 0;
    int32_t torque = 0;
    esc_get_states(&actual_count_, &velocity, &torque);
    set_state(position_state_interface_name_,
              ipe::joint_units::position_to_radians(
                  actual_count_, zero_count_, direction_));

    if (drive_enabled_.load()) {
        uint16_t status = 0;
        uint32_t error = 0;
        esc_get_status_word(&status);
        esc_get_error_codes(&error);
        if ((status & STATUSWORD_STATE_MASK) !=
                STATUSWORD_STATE_OPERATIONENABLED ||
            error != 0) {
            RCLCPP_ERROR(get_logger(),
                         "CSP drive left Operation enabled: status=0x%04x error=0x%08x",
                         status, error);
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IpeCspSystem::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!commanding_.load() || !drive_enabled_.load())
        return hardware_interface::return_type::OK;

    const double requested_radians =
        get_command<double>(position_command_interface_name_);
    int32_t requested_count = 0;
    if (!ipe::joint_units::radians_to_position(
            requested_radians, zero_count_, direction_, &requested_count)) {
        RCLCPP_ERROR(get_logger(), "Rejected non-finite or out-of-range CSP command");
        disable_commanding();
        return hardware_interface::return_type::ERROR;
    }

    const int64_t requested_travel =
        static_cast<int64_t>(requested_count) - activation_origin_count_;
    if (std::llabs(requested_travel) > max_travel_count_) {
        RCLCPP_ERROR(get_logger(),
                     "Rejected CSP command: travel=%ld count exceeds +/- %d count",
                     static_cast<long>(requested_travel), max_travel_count_);
        disable_commanding();
        return hardware_interface::return_type::ERROR;
    }

    const int64_t following_error =
        static_cast<int64_t>(applied_command_count_) - actual_count_;
    if (std::llabs(following_error) > max_following_error_count_) {
        RCLCPP_ERROR(get_logger(),
                     "CSP following error=%ld count exceeds %d count",
                     static_cast<long>(following_error),
                     max_following_error_count_);
        disable_commanding();
        return hardware_interface::return_type::ERROR;
    }

    if (requested_count != applied_command_count_) {
        if (!motion_active_) {
            if (ecatm_arm_csp_motion() < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to arm CSP trigger edge");
                disable_commanding();
                return hardware_interface::return_type::ERROR;
            }
            motion_active_ = true;
            stable_cycles_ = 0;
        }
        applied_command_count_ = slew_toward(
            applied_command_count_, requested_count, max_step_count_);
        ecatm_control(&applied_command_count_);
        if (stable_cycles_ == 0 && ecatm_commit_csp_motion() < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to commit CSP trigger edge");
            disable_commanding();
            return hardware_interface::return_type::ERROR;
        }
        stable_cycles_ = 1;
    } else {
        ecatm_control(&applied_command_count_);
        if (motion_active_ &&
            std::llabs(static_cast<int64_t>(requested_count) - actual_count_) <=
                kTargetToleranceCount) {
            if (++stable_cycles_ >= kTargetStableCycles) {
                motion_active_ = false;
                stable_cycles_ = 0;
            }
        } else if (motion_active_) {
            stable_cycles_ = 1;
        }
    }
    return hardware_interface::return_type::OK;
}

void IpeCspSystem::stop_master() {
    disable_commanding();
    if (master_started_) {
        ecatm_stop();
        master_started_ = false;
    }
    hardware_active_ = false;
}

}  // namespace ipe_ros2_control

PLUGINLIB_EXPORT_CLASS(ipe_ros2_control::IpeCspSystem,
                       hardware_interface::SystemInterface)
