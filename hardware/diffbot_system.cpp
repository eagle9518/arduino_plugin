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

#include "arduino_plugin/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace arduino_plugin
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Drive Wheels
  cfg_.drive_wheel_names.push_back(info_.hardware_parameters["FL_wheel_name"]);
  cfg_.drive_wheel_names.push_back(info_.hardware_parameters["BL_wheel_name"]);
  cfg_.drive_wheel_names.push_back(info_.hardware_parameters["BR_wheel_name"]);
  cfg_.drive_wheel_names.push_back(info_.hardware_parameters["FR_wheel_name"]);

  // Shooter Wheel
  cfg_.shooter_wheel_names.push_back(info_.hardware_parameters["left_shooter_wheel_name"]);
  cfg_.shooter_wheel_names.push_back(info_.hardware_parameters["right_shooter_wheel_name"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Shooter Servos
  cfg_.shooter_servo_names.push_back(info_.hardware_parameters["yaw_shooter_servo_name"]);
  cfg_.shooter_servo_names.push_back(info_.hardware_parameters["pitch_shooter_servo_name"]);

  // General
  cfg_.device = info_.hardware_parameters["device"]; // usually ttyACM0
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  for (size_t i = 0; i < cfg_.drive_wheel_names.size(); ++i)
  {
    drive_wheels_[i].setup(cfg_.drive_wheel_names[i]);
  }
  for (size_t i = 0; i < cfg_.shooter_wheel_names.size(); ++i)
  {
    shooter_wheels_[i].setup(cfg_.shooter_wheel_names[i], cfg_.enc_counts_per_rev);
  }
  for (size_t i = 0; i < cfg_.shooter_servo_names.size(); ++i)
  {
    shooter_servos_[i].setup(cfg_.shooter_servo_names[i]);
  }

  // You can add a validation for-loop here if needed. Since we're using
  // motors + servos it's easier for now to just skip that part

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < cfg_.drive_wheel_names.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.drive_wheel_names[i], hardware_interface::HW_IF_POSITION, &(drive_wheels_[i]).pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.drive_wheel_names[i], hardware_interface::HW_IF_VELOCITY, &(drive_wheels_[i]).vel));
  }
  for (size_t i = 0; i < cfg_.shooter_wheel_names.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.shooter_wheel_names[i], hardware_interface::HW_IF_VELOCITY, &(shooter_wheels_[i]).vel));
  }
  for (size_t i = 0; i < cfg_.shooter_servo_names.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      cfg_.shooter_servo_names[i], hardware_interface::HW_IF_POSITION, &(shooter_servos_[i]).pos));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < cfg_.drive_wheel_names.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      cfg_.drive_wheel_names[i], hardware_interface::HW_IF_VELOCITY, &(drive_wheels_[i]).cmd));
  }
  for (size_t i = 0; i < cfg_.shooter_wheel_names.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      cfg_.shooter_wheel_names[i], hardware_interface::HW_IF_VELOCITY, &(shooter_wheels_[i]).cmd));
  }
  for (size_t i = 0; i < cfg_.shooter_servo_names.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      cfg_.shooter_servo_names[i], hardware_interface::HW_IF_POSITION, &(shooter_servos_[i]).cmd));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // comms_.read_encoder_values(drive_wheels_[0].enc, drive_wheels_[1].enc, drive_wheels_[2].enc, drive_wheels_[3].enc);

  double delta_seconds = period.seconds();

  for (size_t i = 0; i < cfg_.drive_wheel_names.size(); ++i)
  {
    double pos_prev = drive_wheels_[i].pos;
    drive_wheels_[i].pos = drive_wheels_[i].calc_enc_angle();
    drive_wheels_[i].vel = (drive_wheels_[i].pos - pos_prev) / delta_seconds;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type arduino_plugin ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  // commands = {FL_drive_wheel_vel, BL_drive_wheel_vel, BR_wheel_vel, 
  //             FR_wheel_vel, left_shooter_wheel_vel, right_shooter_wheel_vel,
  //             yaw_servo_pos, pitch_servo_pos}
  float commands[8];
  for (size_t i = 0; i < sizeof(commands) / sizeof(float); ++i)
  {
    if (i < cfg_.drive_wheel_names.size()) {
      commands[i] = drive_wheels_[i].cmd;
    } else if (i < cfg_.drive_wheel_names.size() + cfg_.shooter_wheel_names.size()) {
      commands[i] = shooter_wheels_[i].cmd;
    } else {
      commands[i] = shooter_servos_[i].cmd;
    }
  }
  comms_.send_command(commands, sizeof(commands) / sizeof(float));

  return hardware_interface::return_type::OK;
}

}  // namespace arduino_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arduino_plugin::DiffBotSystemHardware, hardware_interface::SystemInterface)
