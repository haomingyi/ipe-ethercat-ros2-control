#ifndef IPE_CONTROLLERS__CST_IMPEDANCE_CONTROLLER_HPP_
#define IPE_CONTROLLERS__CST_IMPEDANCE_CONTROLLER_HPP_

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

namespace ipe_controllers {

class CstImpedanceController final : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Reference {
    double position{0.0};
    double velocity{0.0};
    double feedforward_raw{0.0};
    std::chrono::steady_clock::time_point received{};
  };

  void reference_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void publish_status();

  std::string joint_name_{"ipe_joint"};
  double kp_raw_per_rad_{0.0};
  double kd_raw_per_rad_s_{0.0};
  double breakaway_raw_{0.0};
  double position_deadband_rad_{0.0};
  double breakaway_velocity_threshold_rad_s_{0.0};
  double max_command_raw_{0.0};
  double max_position_error_rad_{0.0};
  double reference_timeout_sec_{0.0};

  realtime_tools::RealtimeBuffer<Reference> reference_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_subscription_;
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<double> status_position_{0.0};
  std::atomic<double> status_velocity_{0.0};
  std::atomic<double> status_torque_actual_raw_{0.0};
  std::atomic<double> status_reference_position_{0.0};
  std::atomic<double> status_reference_velocity_{0.0};
  std::atomic<double> status_command_raw_{0.0};
  std::atomic<bool> status_reference_valid_{false};
};

}  // namespace ipe_controllers

#endif  // IPE_CONTROLLERS__CST_IMPEDANCE_CONTROLLER_HPP_
