// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
#define GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_

#define VELOCITY_PID_PARAMS_PREFIX "vel_"
#define POSITION_PID_PARAMS_PREFIX "pos_"

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "control_toolbox/pid.hpp"
#include "gazebo_ros2_control/gazebo_system_interface.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gazebo_ros2_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration untuk private data class (OOP)
class GazeboSystemPrivate;

// Kelas utama hardware interface untuk ros2_control di Gazebo
class GazeboSystem : public GazeboSystemInterface
{
public:
  // Inisialisasi hardware interface dari URDF/Xacro (dipanggil oleh ros2_control)
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  // BENAR: return by value, sesuai base class ROS2 control
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Lifecycle: aktifkan hardware interface
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Lifecycle: nonaktifkan hardware interface
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Mode switch (jika ingin ganti mode kontrol, misal velocity ke position)
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Baca data dari Gazebo ke ros2_control (posisi, velocity, effort)
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Kirim command dari ros2_control ke Gazebo (velocity, position, effort)
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Inisialisasi plugin Gazebo (dipanggil oleh plugin loader)
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

private:
  // Register semua joint dari URDF/Xacro ke vector internal
  void registerJoints(
    const hardware_interface::HardwareInfo & hardware_info,
    gazebo::physics::ModelPtr parent_model);

  // Register semua sensor dari URDF/Xacro ke vector internal
  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info,
    gazebo::physics::ModelPtr parent_model);

  // Ekstrak parameter PID dari YAML/parameter server (jika ingin tuning PID)
  bool extractPID(
    const std::string & prefix,
    const hardware_interface::ComponentInfo & joint_info, control_toolbox::Pid & pid);

  // Ekstrak parameter PID dari parameter ROS2 (jika ingin tuning PID)
  bool extractPIDFromParameters(
    const std::string & control_mode, const std::string & joint_name, control_toolbox::Pid & pid);

  // Monitoring health hardware (publish ke topic ROS2)
  void publish_health_status(const std::string & status);

  /// \brief Private data class (semua vector dan pointer internal)
  std::unique_ptr<GazeboSystemPrivate> dataPtr;
};

}  // namespace gazebo_ros2_control

#endif  // GAZEBO_ROS2_CONTROL__GAZEBO_SYSTEM_HPP_
