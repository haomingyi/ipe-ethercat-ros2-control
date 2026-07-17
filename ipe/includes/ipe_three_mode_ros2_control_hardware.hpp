#ifndef IPE_THREE_MODE_ROS2_CONTROL_HARDWARE_HPP
#define IPE_THREE_MODE_ROS2_CONTROL_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace ipe_ros2_control {

class IpeThreeModeSystem final : public hardware_interface::SystemInterface {
public:
    ~IpeThreeModeSystem() override;

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
    enum class Mode : uint8_t { None, Csp, Csv, Cst };

    Mode requested_mode(const std::vector<std::string>& interfaces,
                        bool* multiple) const;
    bool enable_mode(Mode mode);
    void disable_commanding();
    void stop_master();
    const char* mode_name(Mode mode) const;

    bool master_started_{false};
    bool hardware_active_{false};
    std::atomic_bool drive_enabled_{false};
    std::atomic_bool commanding_{false};
    std::atomic<Mode> active_mode_{Mode::None};
    Mode pending_mode_{Mode::None};
    bool pending_stop_{false};
    bool motion_active_{false};

    std::string interface_name_;
    std::string joint_name_;
    std::string position_state_interface_name_;
    std::string velocity_si_state_interface_name_;
    std::string velocity_state_interface_name_;
    std::string torque_state_interface_name_;
    std::string position_command_interface_name_;
    std::string velocity_command_interface_name_;
    std::string torque_command_interface_name_;

    int32_t zero_count_{0};
    int direction_{1};
    int raw_direction_{1};
    double velocity_raw_to_rad_s_{2.3731138426448658e-7};
    int32_t csp_max_travel_count_{26214400};
    int32_t csp_max_step_count_{1200};
    int32_t csp_max_following_error_count_{16384};
    int32_t csv_max_raw_{13107200};
    int32_t csv_max_step_raw_{10000};
    int32_t csv_max_travel_count_{0};
    int32_t cst_max_raw_{300};
    int32_t cst_max_step_raw_{1};
    int32_t cst_max_travel_count_{0};
    int32_t cst_slew_interval_cycles_{2};
    int32_t rated_velocity_raw_{13107200};
    int32_t cst_slew_cycle_{0};

    int32_t activation_origin_count_{0};
    int32_t actual_count_{0};
    int32_t actual_velocity_{0};
    int32_t actual_torque_{0};
    int32_t applied_command_{0};
};

}  // namespace ipe_ros2_control

#endif
