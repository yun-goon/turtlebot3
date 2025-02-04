// Copyright 2022 ROBOTIS CO., LTD.
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
// Author: Darby Lim

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtlebot3_hardware/turtlebot3_system.hpp"

namespace robotis
{
namespace turtlebot3_hardware
{
auto logger = rclcpp::get_logger("turtlebot3");
hardware_interface::CallbackReturn TurtleBot3SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  opencr_ = std::make_unique<OpenCR>(id_);
  if (opencr_->open_port(usb_port_)) {
    RCLCPP_INFO(logger, "Succeeded to open port");
  } else {
    RCLCPP_FATAL(logger, "Failed to open port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->set_baud_rate(baud_rate_)) {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  int32_t model_number = opencr_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

  if (opencr_->is_connect_wheels()) {
    RCLCPP_INFO(logger, "Connected wheels");
  } else {
    RCLCPP_FATAL(logger, "Not connected wheels");
    return hardware_interface::CallbackReturn::ERROR;
  }

  dxl_wheel_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

  opencr_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size() +
    info_.sensors[1].state_interfaces.size(),
    0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TurtleBot3SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

  for (uint8_t i = 0, k = 0; i < info_.sensors.size(); i++) {
    for (uint8_t j = 0; j < info_.sensors[i].state_interfaces.size(); j++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[i].name,
          info_.sensors[i].state_interfaces[j].name,
          &opencr_sensor_states_[k++])
      );
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtleBot3SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[1]));

  return command_interfaces;
}

hardware_interface::CallbackReturn TurtleBot3SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for start");
  opencr_->send_heartbeat(heartbeat_++);

  RCLCPP_INFO(logger, "Wait for IMU re-calibration");
  opencr_->imu_recalibration();
  rclcpp::sleep_for(std::chrono::seconds(3));

  opencr_->send_heartbeat(heartbeat_++);
  RCLCPP_INFO(logger, "Wheels torque ON");
  opencr_->wheels_torque(opencr::ON);

  RCLCPP_INFO(logger, "System starting");
  opencr_->play_sound(opencr::SOUND::ASCENDING);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurtleBot3SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for stop");
  opencr_->play_sound(opencr::SOUND::DESCENDING);

  RCLCPP_INFO(logger, "System stopped");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TurtleBot3SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to read wheels states");

  if (opencr_->read_all() == false) {
    RCLCPP_WARN(logger, "Failed to read all control table");
  }

  dxl_positions_[0] = opencr_->get_wheel_positions()[opencr::wheels::LEFT];
  dxl_velocities_[0] = opencr_->get_wheel_velocities()[opencr::wheels::LEFT];

  dxl_positions_[1] = opencr_->get_wheel_positions()[opencr::wheels::RIGHT];
  dxl_velocities_[1] = opencr_->get_wheel_velocities()[opencr::wheels::RIGHT];

  opencr_sensor_states_[0] = opencr_->get_imu().orientation.x;
  opencr_sensor_states_[1] = opencr_->get_imu().orientation.y;
  opencr_sensor_states_[2] = opencr_->get_imu().orientation.z;
  opencr_sensor_states_[3] = opencr_->get_imu().orientation.w;

  opencr_sensor_states_[4] = opencr_->get_imu().angular_velocity.x;
  opencr_sensor_states_[5] = opencr_->get_imu().angular_velocity.y;
  opencr_sensor_states_[6] = opencr_->get_imu().angular_velocity.z;

  opencr_sensor_states_[7] = opencr_->get_imu().linear_acceleration.x;
  opencr_sensor_states_[8] = opencr_->get_imu().linear_acceleration.y;
  opencr_sensor_states_[9] = opencr_->get_imu().linear_acceleration.z;

  opencr_sensor_states_[10] = opencr_->get_battery().voltage;
  opencr_sensor_states_[11] = opencr_->get_battery().percentage;
  opencr_sensor_states_[12] = opencr_->get_battery().design_capacity;
  opencr_sensor_states_[13] = opencr_->get_battery().present;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to write wheels commands");
  opencr_->send_heartbeat(heartbeat_++);

  if (opencr_->set_wheel_velocities(dxl_wheel_commands_) == false) {
    RCLCPP_ERROR(logger, "Can't control wheels");
  }

  return hardware_interface::return_type::OK;
}
}  // namespace turtlebot3_hardware
}  // namespace robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::turtlebot3_hardware::TurtleBot3SystemHardware,
  hardware_interface::SystemInterface)
