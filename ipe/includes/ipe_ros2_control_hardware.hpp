#ifndef IPE_ROS2_CONTROL_HARDWARE_HPP
#define IPE_ROS2_CONTROL_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <cstdint>
#include <cstddef>
#include <string>

namespace ipe_ros2_control {

class IpeReadOnlySystem final : public hardware_interface::SystemInterface {
public:
    ~IpeReadOnlySystem() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    void stop_master();

    bool master_started_{false};
    std::string interface_name_;
    std::string joint_name_;
    std::string position_interface_name_;
    int32_t zero_count_{0};
    int direction_{1};
    std::size_t unhealthy_read_cycles_{0};
};

}  // namespace ipe_ros2_control

#endif
