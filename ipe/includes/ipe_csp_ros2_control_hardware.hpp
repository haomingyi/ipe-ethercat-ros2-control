#ifndef IPE_CSP_ROS2_CONTROL_HARDWARE_HPP
#define IPE_CSP_ROS2_CONTROL_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace ipe_ros2_control {

class IpeCspSystem final : public hardware_interface::SystemInterface {
public:
    ~IpeCspSystem() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    bool enable_commanding();
    void disable_commanding();
    void stop_master();
    bool command_interface_requested(
        const std::vector<std::string>& interfaces) const;

    bool master_started_{false};
    bool hardware_active_{false};
    std::atomic_bool drive_enabled_{false};
    std::atomic_bool commanding_{false};
    bool pending_start_{false};
    bool pending_stop_{false};
    bool motion_active_{false};
    int stable_cycles_{0};

    std::string interface_name_;
    std::string joint_name_;
    std::string position_state_interface_name_;
    std::string position_command_interface_name_;
    int32_t zero_count_{0};
    int direction_{1};
    int32_t max_travel_count_{1000};
    int32_t max_step_count_{10};
    int32_t max_following_error_count_{200};

    int32_t activation_origin_count_{0};
    int32_t actual_count_{0};
    int32_t applied_command_count_{0};
};

}  // namespace ipe_ros2_control

#endif
