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

#include <boost/asio/read_until.hpp>
#include <chrono>
#include <limits>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robo_inspecto_hardware/motor_hardware.hpp"

using namespace boost::asio;

namespace robo_inspecto_hardware {
hardware_interface::CallbackReturn
MotorHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::ActuatorInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  serialCon = serial_port(io, "/dev/ttyUSB0");
  serialCon.set_option(serial_port_base::baud_rate(115200));

  // set arduino serial settings
  //  serialCon.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  //  serialCon.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  //  serialCon.SetParity(LibSerial::Parity::PARITY_NONE);
  //  serialCon.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

  // Set character size to 8
  serialCon.set_option(serial_port_base::character_size(8));

  // Set flow control to none
  serialCon.set_option(
      serial_port_base::flow_control(serial_port_base::flow_control::none));

  // Set parity to none
  serialCon.set_option(
      serial_port_base::parity(serial_port_base::parity::none));

  // Set stop bits to one
  serialCon.set_option(
      serial_port_base::stop_bits(serial_port_base::stop_bits::one));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // check if serial port is open
  if (!serialCon.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Serial port not open");
    return CallbackReturn::ERROR;
  }

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

  writeSerial(READ_COMMAND);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO(anyone): prepare the robot to stop receiving commands

  serialCon.close();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MotorHardware::read(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) {

  if (!serialCon.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }
  
  writeSerial(READ_COMMAND);
  //async read until
  async_read_until(serialCon, responseData, '\n',
            boost::bind(&MotorHardware::handleRead, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));

  //check if new data is available
  if (!this->newSerialData) {
    return hardware_interface::return_type::OK;
  }
  this->newSerialData = false;

  //read data
  std::string response;
  //move data from buffer to response
  std::istream responseStream(&responseData);
  std::getline(responseStream, response, '\n');
  //clear buffer
  responseData.consume(responseData.size());
  double pos;
  if(!convertResponseToDouble(response, pos)){
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Error converting response to double");
    return hardware_interface::return_type::OK;
  }

  if(pos < 0.0){
    RCLCPP_WARN(rclcpp::get_logger("MotorHardware"), "Position is negative");
    return hardware_interface::return_type::OK;
  }

  motor.pos = pos;

  return hardware_interface::return_type::OK;
}

void MotorHardware::handleRead(const boost::system::error_code& error, std::size_t){
  if (error) {
    // print error msg
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),
                 "Error reading serial port: %s", error.message().c_str());
    return;
  }
  this->newSerialData = true;
}

void MotorHardware::writeSerial(const std::string &command) {
  std::string commandWithPrefix = COMMAND_PREFIX + command + COMMAND_SUFFIX;
  serialCon.write_some(buffer(commandWithPrefix));
}

bool MotorHardware::readSerial(std::string &response) {
  read_until(serialCon, responseData, 'n', error);
  if (error) {
    // print error msg
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),
                 "Error reading serial port: %s", error.message().c_str());
    return false;
  }

  std::istream responseStream(&responseData);
  std::getline(responseStream, response);
  return !response.empty();
}

bool MotorHardware::convertResponseToDouble(const std::string &response,
                                            double &value) {
  // find COMMAND_PREFIX
  size_t prefixPos = response.find(COMMAND_PREFIX);
  if (prefixPos == std::string::npos) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),"Command prefix not found in response: %s", response.c_str());
    return false;
  }
  //get substring after COMMAND_PREFIX
  auto length = response.length() - prefixPos - 1;
  std::string valueString = response.substr(prefixPos + 1, length-1);
  //check if only contains digits or . or -
  auto notDigits = valueString.find_first_not_of("0123456789.-");

  if (notDigits != std::string::npos) {
    //print error and notDigits pos and char
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),"Response contains non digit characters: %s", valueString.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),"Not digit pos: %ld", notDigits);
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),"Not digit char: %c", valueString[notDigits]);

    return false;
  }

  // convert to double
  try {
    value = std::stod(valueString) / 1000.0;
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),"Invalid argument: %s", e.what());
    return false;
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"),"Out of range: %s", e.what());
    return false;
  }

  return true;
}

hardware_interface::return_type
MotorHardware::write(const rclcpp::Time & /*time*/,
                     const rclcpp::Duration & /*period*/) {
  // TODO(anyone): write robot's commands'

  if (!serialCon.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorHardware"), "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }

  

  // check if cmd is different then last cmd
  if (motor.cmd == motor.pos) {
    return hardware_interface::return_type::OK;
  }
  static double lastCmd = -1.0;
  if (motor.cmd == lastCmd) {
    return hardware_interface::return_type::OK;
  }
  lastCmd = motor.cmd;

  //if negative cmd, set to 0
  if (motor.cmd < 0.0) {
    motor.pos = motor.cmd;
    motor.cmd = 0.01;
  }

  // log the cmd
  RCLCPP_INFO(rclcpp::get_logger("MotorHardware"), "cmd: %f", motor.cmd);
  std::string command = WRITE_POSITION_COMMAND + std::to_string(motor.cmd * 1000);
  writeSerial(command);
  writeSerial(READ_COMMAND);

  return hardware_interface::return_type::OK;
}

} // namespace robo_inspecto_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robo_inspecto_hardware::MotorHardware,
                       hardware_interface::ActuatorInterface)
