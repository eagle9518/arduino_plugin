// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
// This code was modified by Logan Wong

#ifndef ARDUINO_PLUGIN_DIFFBOT_SYSTEM_HPP_
#define ARDUINO_PLUGIN_DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_plugin/arduino_comms.hpp"
#include "arduino_plugin/wheel.hpp"
#include "arduino_plugin/servo.hpp"


namespace arduino_plugin
{
// Actuator, Sensor, or System
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{

struct Config
{
  // CCW around robot starting from FL wheel
  std::vector<std::string> drive_wheel_names;
  std::vector<std::string> shooter_wheel_names;
  std::vector<std::string> shooter_servo_names;
  float loop_rate = 0.0;
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  std::string device = "";
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ArduinoComms comms_;
  Config cfg_;
  std::vector<Wheel> drive_wheels_{4};
  std::vector<Wheel> shooter_wheels_{2};
  std::vector<Servo> shooter_servos_{2};
};

}  // namespace arduino_plugin

#endif  // ARDUINO_PLUGIN_DIFFBOT_SYSTEM_HPP_
