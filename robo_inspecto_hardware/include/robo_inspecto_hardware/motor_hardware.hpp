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

#ifndef ROBO_INSPECTO_HARDWARE__MOTOR_HARDWARE_HPP_
#define ROBO_INSPECTO_HARDWARE__MOTOR_HARDWARE_HPP_

#include <cstddef>
#include <string>
#include <vector>


//boost includes
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>



#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robo_inspecto_hardware/visibility_control.h"

namespace robo_inspecto_hardware {
class MotorHardware : public hardware_interface::ActuatorInterface {
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // LibSerial::SerialPort serialCon;
  boost::asio::io_service io;
  boost::asio::serial_port serialCon = boost::asio::serial_port(io);
  
  typedef struct Motor{
    double cmd = 0.0;
    double pos = 0.0;
  } Motor_t;

  Motor_t motor;

  #define COMMAND_PREFIX ";"
  #define COMMAND_SUFFIX "\n"
  #define READ_COMMAND "r"
  #define WRITE_POSITION_COMMAND "p"


  void writeSerial(const std::string &command);
  bool readSerial(std::string &response);

  bool convertResponseToDouble(const std::string &response, double &value);

  //handler for async read
  void handleRead(const boost::system::error_code& error, std::size_t);
    


  boost::system::error_code error;
  boost::asio::streambuf responseData;

  bool newSerialData = false;
};

} // namespace robo_inspecto_hardware

#endif // ROBO_INSPECTO_HARDWARE__MOTOR_HARDWARE_HPP_
