// Copyright 2020 ROS2-Control Development Team
// Licensed under the Apache License, Version 2.0.

#include <errno.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace
{
constexpr int kSchedPriority = 50;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = controller_manager::get_cm_node_options();
  auto node_arguments = options.arguments();
  for (int i = 1; i < argc; ++i)
  {
    if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
    {
      continue;
    }
    node_arguments.push_back(argv[i]);
  }
  options.arguments(node_arguments);

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, "controller_manager", "", options);

  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false);
  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory)
  {
    const auto result = realtime_tools::lock_memory();
    if (!result.first)
    {
      RCLCPP_WARN(cm->get_logger(), "Unable to lock memory: '%s'", result.second.c_str());
    }
  }

  const bool manage_overruns = cm->get_parameter_or<bool>("overruns.manage", true);
  const int priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

  (void)use_sim_time;
  (void)manage_overruns;

  std::thread control_thread(
    [cm, priority]()
    {
      if (!realtime_tools::configure_sched_fifo(priority))
      {
        RCLCPP_WARN(
          cm->get_logger(), "Could not enable FIFO scheduling: <%i>(%s)", errno,
          std::strerror(errno));
      }

      cm->get_clock()->wait_until_started();
      cm->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0 / cm->get_update_rate()));

      const auto period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
      auto previous_time = cm->get_trigger_clock()->now();
      auto next_iteration = std::chrono::steady_clock::now() + period;

      while (rclcpp::ok())
      {
        const auto current_time = cm->get_trigger_clock()->now();
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;
        cm->read(current_time, measured_period);
        cm->update(current_time, measured_period);
        cm->write(current_time, measured_period);
        std::this_thread::sleep_until(next_iteration);
        next_iteration += period;
      }
    });

  executor->add_node(cm);
  executor->spin();
  control_thread.join();
  rclcpp::shutdown();
  return 0;
}
