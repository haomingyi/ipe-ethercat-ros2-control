#include "ecat_motor_master.h"
#include "ipe_joint_units.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

class IpeJointStatePublisher final : public rclcpp::Node {
public:
    IpeJointStatePublisher()
        : Node("ipe_joint_state_publisher") {
        const std::string interface_name =
            declare_parameter<std::string>("interface", "enp130s0");
        joint_name_ = declare_parameter<std::string>("joint_name", "ipe_joint");
        const int64_t direction_parameter =
            declare_parameter<int64_t>("direction", 1);
        const bool startup_zero =
            declare_parameter<bool>("startup_position_as_zero", true);
        const int64_t configured_zero =
            declare_parameter<int64_t>("zero_count", 0);
        const double publish_rate_hz =
            declare_parameter<double>("publish_rate_hz", 50.0);

        if (interface_name.empty() || joint_name_.empty())
            throw std::invalid_argument("interface and joint_name cannot be empty");
        if (!ipe::joint_units::is_valid_direction(
                static_cast<int>(direction_parameter))) {
            throw std::invalid_argument("direction must be 1 or -1");
        }
        if (publish_rate_hz <= 0.0 || publish_rate_hz > 200.0)
            throw std::invalid_argument("publish_rate_hz must be in (0, 200]");
        if (!startup_zero &&
            (configured_zero < std::numeric_limits<int32_t>::min() ||
             configured_zero > std::numeric_limits<int32_t>::max())) {
            throw std::invalid_argument("zero_count is outside int32 range");
        }
        direction_ = static_cast<int>(direction_parameter);

        if (ecatm_set_interface(interface_name.c_str()) < 0)
            throw std::runtime_error("invalid EtherCAT interface name");
        if (ecatm_init_passive("csp", 1) < 0)
            throw std::runtime_error("passive EtherCAT initialization failed");
        master_started_ = true;

        int32_t actual_count = 0;
        int32_t velocity_raw = 0;
        int32_t torque_raw = 0;
        esc_get_states(&actual_count, &velocity_raw, &torque_raw);
        zero_count_ = startup_zero ? actual_count
                                   : static_cast<int32_t>(configured_zero);

        publisher_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS());
        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publish_joint_state(); });

        RCLCPP_INFO(get_logger(),
                    "Passive IPE monitor ready: interface=%s joint=%s "
                    "zero_count=%d direction=%d rate=%.1f Hz; motor was not enabled",
                    interface_name.c_str(), joint_name_.c_str(), zero_count_,
                    direction_, publish_rate_hz);
    }

    ~IpeJointStatePublisher() override {
        if (master_started_)
            ecatm_stop();
    }

private:
    void publish_joint_state() {
        if (!ecatm_is_pdo_healthy()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "PDO is unhealthy; joint state not published");
            return;
        }

        int32_t actual_count = 0;
        int32_t velocity_raw = 0;
        int32_t torque_raw = 0;
        esc_get_states(&actual_count, &velocity_raw, &torque_raw);

        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name.push_back(joint_name_);
        message.position.push_back(ipe::joint_units::position_to_radians(
            actual_count, zero_count_, direction_));
        // Velocity and effort stay empty until the IPE raw-value scales are
        // confirmed. Publishing unknown units as rad/s or N*m would be wrong.
        publisher_->publish(message);
    }

    bool master_started_{false};
    int32_t zero_count_{0};
    int direction_{1};
    std::string joint_name_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<IpeJointStatePublisher>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("ipe_joint_state_publisher"), "%s",
                     exception.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
