#include "ipe_ros2_control_hardware.hpp"

#include "ecat_motor_master.h"
#include "ipe_joint_units.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include <charconv>
#include <system_error>

namespace ipe_ros2_control {
namespace {

// The SOEM PDO loop already requires ten consecutive bad work counters before
// reporting unhealthy.  A non-real-time desktop can still occasionally cross
// that short window.  Since this component is read-only, retain the last good
// position for up to one second (100 controller-manager reads) and fail safe if
// communication does not recover.
constexpr std::size_t kUnhealthyReadLimit = 100;

template <typename T>
bool parse_integer(const std::string& text, T* value) {
    if (!value)
        return false;
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto result = std::from_chars(begin, end, *value);
    return result.ec == std::errc{} && result.ptr == end;
}

}  // namespace

IpeReadOnlySystem::~IpeReadOnlySystem() {
    stop_master();
}

hardware_interface::CallbackReturn IpeReadOnlySystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto& info = get_hardware_info();
    if (info.joints.size() != 1 || info.joints.front().state_interfaces.size() != 1 ||
        info.joints.front().state_interfaces.front().name !=
            hardware_interface::HW_IF_POSITION ||
        !info.joints.front().command_interfaces.empty()) {
        RCLCPP_ERROR(get_logger(),
                     "IPE read-only hardware requires one joint, one position "
                     "state interface, and no command interfaces");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto interface_it = info.hardware_parameters.find("interface");
    const auto zero_it = info.hardware_parameters.find("zero_count");
    const auto direction_it = info.hardware_parameters.find("direction");
    if (interface_it == info.hardware_parameters.end() ||
        zero_it == info.hardware_parameters.end() ||
        direction_it == info.hardware_parameters.end()) {
        RCLCPP_ERROR(get_logger(),
                     "Missing interface, zero_count, or direction hardware parameter");
        return hardware_interface::CallbackReturn::ERROR;
    }

    interface_name_ = interface_it->second;
    joint_name_ = info.joints.front().name;
    position_interface_name_ =
        joint_name_ + "/" + hardware_interface::HW_IF_POSITION;
    if (interface_name_.empty() ||
        !parse_integer(zero_it->second, &zero_count_) ||
        !parse_integer(direction_it->second, &direction_) ||
        !ipe::joint_units::is_valid_direction(direction_)) {
        RCLCPP_ERROR(get_logger(), "Invalid IPE hardware parameters");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(),
                "Initialized read-only IPE hardware metadata: joint=%s "
                "interface=%s zero_count=%d direction=%d",
                joint_name_.c_str(), interface_name_.c_str(), zero_count_,
                direction_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeReadOnlySystem::on_configure(
    const rclcpp_lifecycle::State&) {
    if (ecatm_set_interface(interface_name_.c_str()) < 0 ||
        ecatm_init_passive("csp", 1) < 0) {
        RCLCPP_ERROR(get_logger(), "Passive EtherCAT initialization failed");
        stop_master();
        return hardware_interface::CallbackReturn::ERROR;
    }
    master_started_ = true;
    unhealthy_read_cycles_ = 0;

    int32_t actual_count = 0;
    int32_t velocity_raw = 0;
    int32_t torque_raw = 0;
    esc_get_states(&actual_count, &velocity_raw, &torque_raw);
    set_state(position_interface_name_, ipe::joint_units::position_to_radians(
                                            actual_count, zero_count_, direction_));
    RCLCPP_INFO(get_logger(),
                "Configured passive EtherCAT; motor was not reset or enabled");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeReadOnlySystem::on_activate(
    const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(),
                "Activated read-only state interface; no command interface exists");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeReadOnlySystem::on_cleanup(
    const rclcpp_lifecycle::State&) {
    stop_master();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeReadOnlySystem::on_shutdown(
    const rclcpp_lifecycle::State&) {
    stop_master();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IpeReadOnlySystem::on_error(
    const rclcpp_lifecycle::State&) {
    RCLCPP_ERROR(get_logger(),
                 "Hardware entered the error lifecycle; stopping EtherCAT "
                 "and releasing the network interface");
    stop_master();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IpeReadOnlySystem::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!master_started_)
        return hardware_interface::return_type::ERROR;

    if (!ecatm_is_pdo_healthy()) {
        ++unhealthy_read_cycles_;
        if (unhealthy_read_cycles_ == 1) {
            RCLCPP_WARN(get_logger(),
                        "PDO became unhealthy; retaining the last position "
                        "while waiting up to %zu read cycles for recovery",
                        kUnhealthyReadLimit);
        }
        if (unhealthy_read_cycles_ < kUnhealthyReadLimit)
            return hardware_interface::return_type::OK;

        uint16_t state = 0;
        uint16_t al_status = 0;
        int actual_wkc = 0;
        int required_wkc = 0;
        const int bus_status_result = ecatm_get_bus_status(
            &state, &al_status, &actual_wkc, &required_wkc);
        RCLCPP_ERROR(get_logger(),
                     "PDO remained unhealthy for %zu read cycles; requesting "
                     "hardware deactivation (bus_status=%d state=0x%04x "
                     "AL=0x%04x WKC=%d/%d)",
                     unhealthy_read_cycles_, bus_status_result, state,
                     al_status, actual_wkc, required_wkc);
        return hardware_interface::return_type::ERROR;
    }

    if (unhealthy_read_cycles_ > 0) {
        RCLCPP_INFO(get_logger(), "PDO recovered after %zu unhealthy read cycles",
                    unhealthy_read_cycles_);
        unhealthy_read_cycles_ = 0;
    }

    int32_t actual_count = 0;
    int32_t velocity_raw = 0;
    int32_t torque_raw = 0;
    esc_get_states(&actual_count, &velocity_raw, &torque_raw);
    set_state(position_interface_name_, ipe::joint_units::position_to_radians(
                                            actual_count, zero_count_, direction_));
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IpeReadOnlySystem::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
    // Intentionally empty: the URDF exports no command interface, and this
    // first ros2_control stage must never reset, enable, or move the motor.
    return hardware_interface::return_type::OK;
}

void IpeReadOnlySystem::stop_master() {
    if (!master_started_)
        return;
    ecatm_stop();
    master_started_ = false;
    unhealthy_read_cycles_ = 0;
}

}  // namespace ipe_ros2_control

PLUGINLIB_EXPORT_CLASS(ipe_ros2_control::IpeReadOnlySystem,
                       hardware_interface::SystemInterface)
