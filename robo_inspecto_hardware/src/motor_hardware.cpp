// Copyright (c) 2024, Ewout
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template)
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

#include <limits>
#include <rclcpp/logging.hpp>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robo_inspecto_hardware/motor_hardware.hpp"

namespace robo_inspecto_hardware {
hardware_interface::CallbackReturn
MotorHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::ActuatorInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  
  serialCon.Open("/dev/ttyUSB0");
  serialCon.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

  //set arduino serial settings
  serialCon.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serialCon.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  serialCon.SetParity(LibSerial::Parity::PARITY_NONE);
  serialCon.SetStopBits(LibSerial::StopBits::STOP_BITS_1);  

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // if (!serialCon.IsOpen()) {
  //   RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Serial port not open");
  //   return CallbackReturn::ERROR;
  // }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MotorHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // we only have one joint and one state position

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &this->motor.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MotorHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // we only have one joint and one command position
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &this->motor.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn
MotorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO(anyone): prepare the robot to receive commands

  serialCon.Write("Hello world");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO(anyone): prepare the robot to stop receiving commands

  serialCon.Close();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MotorHardware::read(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) {
  // TODO(anyone): read robot states

  if (!serialCon.IsOpen()) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }
  serialCon.FlushIOBuffers(); // Just in case
  // serialCon.Write(READ_COMMAND);
  //read the response
  std::string response;
  serialCon.DrainWriteBuffer();
  // try {
  //   serialCon.ReadLine(response, '\n', 100);
  // } catch (const LibSerial::ReadTimeout &) {
  //   RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Read timeout");
  //   // return hardware_interface::return_type::ERROR;
  // }

  motor.pos = motor.cmd;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MotorHardware::write(const rclcpp::Time & /*time*/,
                     const rclcpp::Duration & /*period*/) {
  // TODO(anyone): write robot's commands'

  if (!serialCon.IsOpen()) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }

  //check if cmd is different then last cmd
  if (motor.cmd == motor.pos) {
    return hardware_interface::return_type::OK;
  }
  static double lastCmd = -1.0;
  if (motor.cmd == lastCmd) {
    return hardware_interface::return_type::OK;
  }
  lastCmd = motor.cmd;

  //log the cmd
  RCLCPP_INFO(rclcpp::get_logger("MotorHardware"), "cmd: %f", motor.cmd);
  std::string command = WRITE_POSITION_COMMAND + std::to_string(motor.cmd*1000) + "\n";

  serialCon.Write(command);

  return hardware_interface::return_type::OK;
}

} // namespace robo_inspecto_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robo_inspecto_hardware::MotorHardware,
                       hardware_interface::ActuatorInterface)
