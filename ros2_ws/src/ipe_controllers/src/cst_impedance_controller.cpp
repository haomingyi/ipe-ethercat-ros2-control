#include "ipe_controllers/cst_impedance_controller.hpp"
#include "ipe_controllers/impedance_law.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace ipe_controllers {

controller_interface::CallbackReturn CstImpedanceController::on_init() {
  try {
    auto_declare<std::string>("joint_name", "ipe_joint");
    auto_declare<double>("kp_raw_per_rad", 0.0);
    auto_declare<double>("kd_raw_per_rad_s", 0.0);
    auto_declare<double>("breakaway_raw", 0.0);
    auto_declare<double>("position_deadband_rad", 0.0);
    auto_declare<double>("breakaway_velocity_threshold_rad_s", 0.0);
    auto_declare<double>("max_command_raw", 0.0);
    auto_declare<double>("max_position_error_rad", 0.0);
    auto_declare<double>("reference_timeout_sec", 0.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CstImpedanceController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/torque_raw"}};
}

controller_interface::InterfaceConfiguration
CstImpedanceController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/position", joint_name_ + "/velocity",
           joint_name_ + "/torque_raw"}};
}

controller_interface::CallbackReturn CstImpedanceController::on_configure(
    const rclcpp_lifecycle::State &) {
  joint_name_ = get_node()->get_parameter("joint_name").as_string();
  kp_raw_per_rad_ = get_node()->get_parameter("kp_raw_per_rad").as_double();
  kd_raw_per_rad_s_ = get_node()->get_parameter("kd_raw_per_rad_s").as_double();
  breakaway_raw_ = get_node()->get_parameter("breakaway_raw").as_double();
  position_deadband_rad_ =
      get_node()->get_parameter("position_deadband_rad").as_double();
  breakaway_velocity_threshold_rad_s_ =
      get_node()->get_parameter("breakaway_velocity_threshold_rad_s").as_double();
  max_command_raw_ = get_node()->get_parameter("max_command_raw").as_double();
  max_position_error_rad_ =
      get_node()->get_parameter("max_position_error_rad").as_double();
  reference_timeout_sec_ =
      get_node()->get_parameter("reference_timeout_sec").as_double();

  if (joint_name_.empty() || kp_raw_per_rad_ < 0.0 || kd_raw_per_rad_s_ < 0.0 ||
      breakaway_raw_ < 0.0 || breakaway_raw_ > max_command_raw_ ||
      position_deadband_rad_ < 0.0 || position_deadband_rad_ > 0.1 ||
      breakaway_velocity_threshold_rad_s_ <= 0.0 ||
      breakaway_velocity_threshold_rad_s_ > 1.0 ||
      max_command_raw_ <= 0.0 || max_command_raw_ > 300.0 ||
      max_position_error_rad_ <= 0.0 || max_position_error_rad_ > M_PI ||
      reference_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid CST impedance safety parameters");
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "~/reference", rclcpp::SystemDefaultsQoS(),
      std::bind(&CstImpedanceController::reference_callback, this,
                std::placeholders::_1));
  status_publisher_ = get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
      "~/status", rclcpp::SystemDefaultsQoS());
  status_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CstImpedanceController::publish_status, this));

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured CST impedance controller: kp=%.3f raw/rad, "
              "kd=%.3f raw/(rad/s), breakaway=%.1f raw below %.3f rad/s, "
              "deadband=%.4f rad, "
              "limit=%.1f raw",
              kp_raw_per_rad_, kd_raw_per_rad_s_, breakaway_raw_,
              breakaway_velocity_threshold_rad_s_, position_deadband_rad_,
              max_command_raw_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CstImpedanceController::on_activate(
    const rclcpp_lifecycle::State &) {
  if (command_interfaces_.size() != 1 || state_interfaces_.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "CST controller interface assignment is incomplete");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto current_position = state_interfaces_[0].get_optional();
  if (!current_position || !std::isfinite(*current_position) ||
      !command_interfaces_[0].set_value(0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid initial state or command interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  Reference hold;
  hold.position = *current_position;
  hold.received = std::chrono::steady_clock::now();
  reference_buffer_.writeFromNonRT(hold);
  status_command_raw_.store(0.0);
  status_reference_valid_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CstImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  if (!command_interfaces_.empty())
    (void)command_interfaces_[0].set_value(0.0);
  status_command_raw_.store(0.0);
  status_reference_valid_.store(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CstImpedanceController::on_cleanup(
    const rclcpp_lifecycle::State &) {
  reference_subscription_.reset();
  status_timer_.reset();
  status_publisher_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CstImpedanceController::update(
    const rclcpp::Time &, const rclcpp::Duration &) {
  const auto position_value = state_interfaces_[0].get_optional();
  const auto velocity_value = state_interfaces_[1].get_optional();
  const auto torque_actual_value = state_interfaces_[2].get_optional();
  if (!position_value || !velocity_value || !torque_actual_value ||
      !std::isfinite(*position_value) || !std::isfinite(*velocity_value) ||
      !std::isfinite(*torque_actual_value)) {
    (void)command_interfaces_[0].set_value(0.0);
    RCLCPP_ERROR(get_node()->get_logger(),
                 "CST stopped: non-finite or unavailable hardware state");
    return controller_interface::return_type::ERROR;
  }
  const double position = *position_value;
  const double velocity = *velocity_value;
  const double torque_actual_raw = *torque_actual_value;

  const Reference * reference = reference_buffer_.readFromRT();
  const auto now = std::chrono::steady_clock::now();
  const bool reference_valid = reference &&
      std::chrono::duration<double>(now - reference->received).count() <=
          reference_timeout_sec_;

  double command_raw = 0.0;
  if (!reference_valid) {
    (void)command_interfaces_[0].set_value(0.0);
    status_position_.store(position);
    status_velocity_.store(velocity);
    status_torque_actual_raw_.store(torque_actual_raw);
    status_command_raw_.store(0.0);
    status_reference_valid_.store(false);
    const double age = reference
        ? std::chrono::duration<double>(now - reference->received).count()
        : std::numeric_limits<double>::infinity();
    RCLCPP_ERROR(get_node()->get_logger(),
                 "CST stopped: reference timeout (age=%.3f s, limit=%.3f s)",
                 age, reference_timeout_sec_);
    return controller_interface::return_type::ERROR;
  } else {
    const double position_error =
        wrapped_position_error(reference->position, position);
    if (std::abs(position_error) > max_position_error_rad_) {
      (void)command_interfaces_[0].set_value(0.0);
      status_reference_valid_.store(false);
      RCLCPP_ERROR(get_node()->get_logger(),
                   "CST stopped: tracking error %.4f rad exceeds %.4f rad "
                   "(reference=%.4f, actual=%.4f)",
                   position_error, max_position_error_rad_, reference->position,
                   position);
      return controller_interface::return_type::ERROR;
    }
    command_raw = impedance_command_raw(
        reference->position, position, reference->velocity, velocity,
        reference->feedforward_raw, kp_raw_per_rad_, kd_raw_per_rad_s_,
        breakaway_raw_, position_deadband_rad_,
        breakaway_velocity_threshold_rad_s_, max_command_raw_);
    status_reference_position_.store(reference->position);
    status_reference_velocity_.store(reference->velocity);
  }

  if (!command_interfaces_[0].set_value(command_raw)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "CST stopped: torque command interface rejected %.3f raw",
                 command_raw);
    return controller_interface::return_type::ERROR;
  }

  status_position_.store(position);
  status_velocity_.store(velocity);
  status_torque_actual_raw_.store(torque_actual_raw);
  status_command_raw_.store(command_raw);
  status_reference_valid_.store(true);
  return controller_interface::return_type::OK;
}

void CstImpedanceController::reference_callback(
    const sensor_msgs::msg::JointState::SharedPtr message) {
  const auto iterator = std::find(message->name.begin(), message->name.end(), joint_name_);
  if (iterator == message->name.end())
    return;
  const auto index = static_cast<std::size_t>(std::distance(message->name.begin(), iterator));
  if (index >= message->position.size() || !std::isfinite(message->position[index]))
    return;
  Reference reference;
  reference.position = message->position[index];
  if (index < message->velocity.size() && std::isfinite(message->velocity[index]))
    reference.velocity = message->velocity[index];
  reference.received = std::chrono::steady_clock::now();
  reference_buffer_.writeFromNonRT(reference);
}

void CstImpedanceController::publish_status() {
  if (!status_publisher_)
    return;
  control_msgs::msg::DynamicJointState message;
  message.header.stamp = get_node()->now();
  message.joint_names = {joint_name_};
  message.interface_values.resize(1);
  auto & values = message.interface_values.front();
  values.interface_names = {"position", "velocity", "position_reference",
                            "velocity_reference", "torque_command_raw",
                            "torque_actual_raw", "reference_valid"};
  values.values = {status_position_.load(), status_velocity_.load(),
                   status_reference_position_.load(),
                   status_reference_velocity_.load(), status_command_raw_.load(),
                   status_torque_actual_raw_.load(),
                   status_reference_valid_.load() ? 1.0 : 0.0};
  status_publisher_->publish(message);
}

}  // namespace ipe_controllers

PLUGINLIB_EXPORT_CLASS(ipe_controllers::CstImpedanceController,
                       controller_interface::ControllerInterface)
