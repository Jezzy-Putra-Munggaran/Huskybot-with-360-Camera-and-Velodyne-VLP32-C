// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath> // Untuk operasi matematika
#include <limits> // Untuk nilai batas double
#include <map> // Untuk mapping nama interface
#include <memory> // Untuk smart pointer
#include <string> // Untuk string
#include <vector> // Untuk vector
#include <utility> // Untuk std::move
#include <algorithm> // Untuk std::find
#include <chrono> // Untuk timestamp health

#include "control_toolbox/pid.hpp" // PID controller ROS2
#include "gazebo_ros2_control/gazebo_system.hpp" // Header utama class ini
#include "gazebo/sensors/ImuSensor.hh" // Sensor IMU Gazebo
#include "gazebo/sensors/ForceTorqueSensor.hh" // Sensor gaya-torsi Gazebo
#include "gazebo/sensors/SensorManager.hh" // Manajemen sensor Gazebo

#include "hardware_interface/hardware_info.hpp" // Info hardware ROS2 control
#include "hardware_interface/types/hardware_interface_type_values.hpp" // Tipe interface ROS2 control

#include "rclcpp/rclcpp.hpp" // Untuk publisher health
#include "std_msgs/msg/string.hpp" // Untuk pesan health monitoring

// Struktur untuk joint yang mimik joint lain (jika ada)
struct MimicJoint
{
  std::size_t joint_index; // Index joint yang mimik
  std::size_t mimicked_joint_index; // Index joint yang dimimik
  double multiplier = 1.0; // Multiplier nilai mimik
};

// Private data class (OOP, semua data internal disimpan di sini)
class gazebo_ros2_control::GazeboSystemPrivate
{
public:
  GazeboSystemPrivate() = default; // Constructor default
  ~GazeboSystemPrivate() = default; // Destructor default

  size_t n_dof_; // Jumlah degree of freedom (joint)
  size_t n_sensors_; // Jumlah sensor
  gazebo::physics::ModelPtr parent_model_; // Pointer ke model Gazebo
  rclcpp::Time last_update_sim_time_ros_; // Waktu update terakhir
  std::vector<std::string> joint_names_; // Nama semua joint (harus sama dengan Xacro/YAML)
  std::vector<GazeboSystemInterface::ControlMethod> joint_control_methods_; // Metode kontrol per joint
  std::vector<bool> is_joint_actuated_; // Apakah joint bisa dikontrol
  std::vector<gazebo::physics::JointPtr> sim_joints_; // Pointer ke joint Gazebo
  std::vector<double> joint_position_; // Posisi joint
  std::vector<double> joint_velocity_; // Kecepatan joint
  std::vector<double> joint_effort_; // Gaya/torque joint
  std::vector<double> joint_position_cmd_; // Command posisi
  std::vector<double> joint_velocity_cmd_; // Command kecepatan
  std::vector<double> joint_effort_cmd_; // Command gaya/torque
  std::vector<control_toolbox::Pid> vel_pid; // PID velocity (jika ingin tuning)
  std::vector<control_toolbox::Pid> pos_pid; // PID position (jika ingin tuning)
  std::vector<bool> is_pos_pid; // Apakah pakai PID posisi
  std::vector<bool> is_vel_pid; // Apakah pakai PID velocity
  std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_; // Pointer ke sensor IMU
  std::vector<std::array<double, 10>> imu_sensor_data_; // Data sensor IMU
  std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_; // Pointer ke sensor gaya-torsi (belum diimplementasi penuh)
  std::vector<std::array<double, 6>> ft_sensor_data_; // Data sensor gaya-torsi
  std::vector<hardware_interface::StateInterface> state_interfaces_; // State interface ROS2 control
  std::vector<hardware_interface::CommandInterface> command_interfaces_; // Command interface ROS2 control
  std::vector<MimicJoint> mimic_joints_; // Daftar joint yang mimik (jika ada)
  bool hold_joints_ = true; // Apakah joint di-hold saat idle
  GazeboSystemInterface::ControlMethod_ position_control_method_ =
    GazeboSystemInterface::ControlMethod_::POSITION; // Default control method posisi
  GazeboSystemInterface::ControlMethod_ velocity_control_method_ =
    GazeboSystemInterface::ControlMethod_::VELOCITY; // Default control method velocity

  // Health monitoring
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_; // Publisher status hardware
  std::string last_health_status_; // Status terakhir
  rclcpp::Clock clock_; // Clock ROS2
};

namespace gazebo_ros2_control
{

// ===================== INISIALISASI DAN ERROR HANDLING =====================
bool GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  this->dataPtr = std::make_unique<GazeboSystemPrivate>(); // Inisialisasi data private
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time(); // Reset waktu update

  this->nh_ = model_nh; // Simpan node handle
  this->dataPtr->parent_model_ = parent_model; // Simpan pointer model

  // Error handling: cek pointer model
  if (!parent_model) {
    RCLCPP_ERROR(this->nh_->get_logger(), "Parent model pointer is null!");
    return false;
  }

  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
  if (!physics) {
    RCLCPP_ERROR(this->nh_->get_logger(), "Physics engine pointer is null!");
    return false;
  }

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  // Error handling parameter hold_joints
  try {
    this->dataPtr->hold_joints_ = this->nh_->get_parameter("hold_joints").as_bool();
  } catch (...) {
    RCLCPP_WARN(this->nh_->get_logger(), "Parameter 'hold_joints' not set, using default true.");
    this->dataPtr->hold_joints_ = true;
  }

  // Health publisher
  this->dataPtr->health_pub_ = this->nh_->create_publisher<std_msgs::msg::String>("hardware_status", 10);

  registerJoints(hardware_info, parent_model); // Register semua joint
  registerSensors(hardware_info, parent_model); // Register semua sensor

  // Validasi konsistensi joint URDF/Xacro vs YAML controller
  std::vector<std::string> urdf_joints;
  for (const auto & joint : hardware_info.joints) urdf_joints.push_back(joint.name);
  std::vector<std::string> model_joints;
  for (unsigned int i = 0; i < parent_model->GetJointCount(); ++i)
    model_joints.push_back(parent_model->GetJoints()[i]->GetName());

  for (const auto & joint_name : urdf_joints) {
    if (std::find(model_joints.begin(), model_joints.end(), joint_name) == model_joints.end()) {
      RCLCPP_ERROR(this->nh_->get_logger(), "Joint '%s' ada di URDF/Xacro tapi tidak ada di model Gazebo!", joint_name.c_str());
      publish_health_status("ERROR: Joint " + joint_name + " missing in Gazebo model");
      return false;
    }
  }

  if (this->dataPtr->n_dof_ == 0 && this->dataPtr->n_sensors_ == 0) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No joint or sensor available in model!");
    publish_health_status("ERROR: No joint or sensor available in model!");
    return false;
  }

  publish_health_status("OK: Hardware initialized");
  return true;
}

// ===================== REGISTER JOINT (KONSISTEN DENGAN XACRO/YAML) =====================
void GazeboSystem::registerJoints(
  const hardware_interface::HardwareInfo & hardware_info,
  gazebo::physics::ModelPtr parent_model)
{
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->joint_names_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_control_methods_.resize(this->dataPtr->n_dof_);
  this->dataPtr->is_joint_actuated_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->vel_pid.resize(this->dataPtr->n_dof_);
  this->dataPtr->pos_pid.resize(this->dataPtr->n_dof_);
  this->dataPtr->is_pos_pid.resize(this->dataPtr->n_dof_);
  this->dataPtr->is_vel_pid.resize(this->dataPtr->n_dof_);
  this->dataPtr->sim_joints_.resize(this->dataPtr->n_dof_);

  std::vector<std::string> missing_joints;
  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    const std::string joint_name = this->dataPtr->joint_names_[j] = joint_info.name;

    this->dataPtr->sim_joints_[j] = parent_model->GetJoint(joint_name);

    if (!this->dataPtr->sim_joints_[j]) {
      RCLCPP_ERROR(this->nh_->get_logger(), "Joint '%s' tidak ditemukan di model Gazebo!", joint_name.c_str());
      missing_joints.push_back(joint_name);
      publish_health_status("ERROR: Joint " + joint_name + " not found in Gazebo model");
      continue;
    }

    // Register state interface
    for (const auto & iface : joint_info.state_interfaces) {
      if (iface.name == "position") {
        this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joint_position_[j]);
      }
      if (iface.name == "velocity") {
        this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joint_velocity_[j]);
      }
      if (iface.name == "effort") {
        this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joint_effort_[j]);
      }
    }

    // Register command interface
    for (const auto & iface : joint_info.command_interfaces) {
      if (iface.name == "position") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joint_position_cmd_[j]);
      }
      if (iface.name == "velocity") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joint_velocity_cmd_[j]);
      }
      if (iface.name == "effort") {
        this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joint_effort_cmd_[j]);
      }
    }

    this->dataPtr->is_joint_actuated_[j] = (joint_info.command_interfaces.size() > 0);
  }

  if (!missing_joints.empty()) {
    std::string msg = "Joints gagal ditemukan di Gazebo: ";
    for (const auto& jn : missing_joints) msg += jn + " ";
    RCLCPP_ERROR(this->nh_->get_logger(), "%s", msg.c_str());
  }

  int valid_joints = 0;
  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    if (this->dataPtr->sim_joints_[j]) valid_joints++;
  }
  RCLCPP_INFO(rclcpp::get_logger("GazeboSystem"),
    "registerJoints: n_dof_=%zu, valid_joints=%d, state_interfaces_=%zu, command_interfaces_=%zu",
    this->dataPtr->n_dof_,
    valid_joints,
    this->dataPtr->state_interfaces_.size(),
    this->dataPtr->command_interfaces_.size());
}

// ===================== REGISTER SENSOR (IMU, GPS, KAMERA, LIDAR) =====================
void GazeboSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info,
  gazebo::physics::ModelPtr parent_model)
{
  size_t n_sensors = hardware_info.sensors.size();
  for (unsigned int j = 0; j < n_sensors; j++) {
    const auto & component = hardware_info.sensors[j];
    std::string sensor_name = component.name;

    // Cari sensor di Gazebo
    gazebo::sensors::SensorPtr simsensor = gazebo::sensors::SensorManager::Instance()->GetSensor(sensor_name);
    if (!simsensor) {
      RCLCPP_WARN(this->nh_->get_logger(), "Sensor '%s' tidak ditemukan di model Gazebo!", sensor_name.c_str());
      publish_health_status("WARN: Sensor " + sensor_name + " not found in Gazebo model");
      continue;
    }
    if (simsensor->Type() == "imu") {
      auto imu_sensor = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(simsensor);
      if (imu_sensor) {
        this->dataPtr->sim_imu_sensors_.push_back(imu_sensor);
      }
    }
    if (simsensor->Type() == "camera") {
      // TODO: Implementasi handle kamera jika ingin publish ke ROS2
      RCLCPP_INFO(this->nh_->get_logger(), "Camera sensor '%s' terdeteksi (belum di-handle di plugin ini)", sensor_name.c_str());
    }
    if (simsensor->Type() == "ray" || simsensor->Type() == "gpu_ray") {
      // TODO: Implementasi handle LiDAR jika ingin publish ke ROS2
      RCLCPP_INFO(this->nh_->get_logger(), "LiDAR sensor '%s' terdeteksi (belum di-handle di plugin ini)", sensor_name.c_str());
    }
    if (simsensor->Type() == "gps") {
      // TODO: Implementasi handle GPS jika ingin publish ke ROS2
      RCLCPP_INFO(this->nh_->get_logger(), "GPS sensor '%s' terdeteksi (belum di-handle di plugin ini)", sensor_name.c_str());
    }
    // Tambahkan sensor lain jika ada (FT, dsb)
  }
  this->dataPtr->imu_sensor_data_.resize(this->dataPtr->sim_imu_sensors_.size());
  this->dataPtr->n_sensors_ = this->dataPtr->sim_imu_sensors_.size();
}

// ===================== MONITORING HEALTH HARDWARE =====================
void GazeboSystem::publish_health_status(const std::string & status)
{
  RCLCPP_DEBUG(rclcpp::get_logger("GazeboSystem"), 
    "publish_health_status dipanggil. nh_=%p, dataPtr=%p, health_pub_=%p, status='%s'", 
    static_cast<void*>(this->nh_.get()), 
    static_cast<void*>(this->dataPtr.get()), 
    static_cast<void*>(this->dataPtr ? this->dataPtr->health_pub_.get() : nullptr),
    status.c_str());

  if (!this->nh_ || !this->dataPtr || !this->dataPtr->health_pub_) return;
  if (status == this->dataPtr->last_health_status_) return; // Hindari spam
  auto msg = std_msgs::msg::String();
  msg.data = "[" + std::to_string(this->dataPtr->clock_.now().seconds()) + "] " + status;
  this->dataPtr->health_pub_->publish(msg);
  this->dataPtr->last_health_status_ = status;
}

// ===================== EXPORT INTERFACE (DENGAN LOGGING) =====================
std::vector<hardware_interface::StateInterface>
GazeboSystem::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("GazeboSystem"),
    "export_state_interfaces: joint_names_.size()=%zu sim_joints_.size()=%zu joint_position_.size()=%zu joint_velocity_.size()=%zu joint_effort_.size()=%zu state_interfaces_.size()=%zu",
    this->dataPtr->joint_names_.size(),
    this->dataPtr->sim_joints_.size(),
    this->dataPtr->joint_position_.size(),
    this->dataPtr->joint_velocity_.size(),
    this->dataPtr->joint_effort_.size(),
    this->dataPtr->state_interfaces_.size());

  // Validasi vector tidak kosong dan ukurannya konsisten
  if (this->dataPtr->joint_names_.empty() || this->dataPtr->sim_joints_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("GazeboSystem"), "export_state_interfaces: joint_names_ atau sim_joints_ kosong! Plugin akan crash.");
    throw std::runtime_error("export_state_interfaces: joint_names_ atau sim_joints_ kosong!");
  }
  if (this->dataPtr->joint_names_.size() != this->dataPtr->sim_joints_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("GazeboSystem"), "export_state_interfaces: Ukuran joint_names_ dan sim_joints_ tidak sama!");
    throw std::runtime_error("export_state_interfaces: Ukuran joint_names_ dan sim_joints_ tidak sama!");
  }
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSystem::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("GazeboSystem"),
    "export_command_interfaces: joint_names_.size()=%zu sim_joints_.size()=%zu",
    this->dataPtr->joint_names_.size(), this->dataPtr->sim_joints_.size());
  return std::move(this->dataPtr->command_interfaces_);
}

// ===================== CALLBACK LIFECYCLE =====================
CallbackReturn GazeboSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    publish_health_status("ERROR: on_init failed");
    return CallbackReturn::ERROR;
  }
  publish_health_status("OK: on_init success");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSystem::on_activate(const rclcpp_lifecycle::State &)
{
  publish_health_status("OK: on_activate success");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  publish_health_status("OK: on_deactivate success");
  return CallbackReturn::SUCCESS;
}

// ===================== READ/WRITE DATA JOINT DAN SENSOR =====================
hardware_interface::return_type GazeboSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  bool all_ok = true;
  // Baca posisi, kecepatan, effort dari joint Gazebo
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      this->dataPtr->joint_position_[j] = this->dataPtr->sim_joints_[j]->Position(0);
      this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
      this->dataPtr->joint_effort_[j] = this->dataPtr->sim_joints_[j]->GetForce(0u);
    } else {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("GazeboSystem"), "Joint '%s' pointer NULL saat read!", this->dataPtr->joint_names_[j].c_str());
      publish_health_status("ERROR: Joint " + this->dataPtr->joint_names_[j] + " pointer NULL saat read");
      all_ok = false;
    }
  }
  // Baca data IMU jika ada
  for (unsigned int j = 0; j < this->dataPtr->sim_imu_sensors_.size(); j++) {
    auto sim_imu = this->dataPtr->sim_imu_sensors_[j];
    if (sim_imu) {
      this->dataPtr->imu_sensor_data_[j][0] = sim_imu->Orientation().X();
      this->dataPtr->imu_sensor_data_[j][1] = sim_imu->Orientation().Y();
      this->dataPtr->imu_sensor_data_[j][2] = sim_imu->Orientation().Z();
      this->dataPtr->imu_sensor_data_[j][3] = sim_imu->Orientation().W();
      this->dataPtr->imu_sensor_data_[j][4] = sim_imu->AngularVelocity().X();
      this->dataPtr->imu_sensor_data_[j][5] = sim_imu->AngularVelocity().Y();
      this->dataPtr->imu_sensor_data_[j][6] = sim_imu->AngularVelocity().Z();
      this->dataPtr->imu_sensor_data_[j][7] = sim_imu->LinearAcceleration().X();
      this->dataPtr->imu_sensor_data_[j][8] = sim_imu->LinearAcceleration().Y();
      this->dataPtr->imu_sensor_data_[j][9] = sim_imu->LinearAcceleration().Z();
    } else {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("GazeboSystem"), "IMU sensor pointer NULL saat read!");
      publish_health_status("ERROR: IMU sensor pointer NULL saat read");
      all_ok = false;
    }
  }
  if (all_ok) publish_health_status("OK: read success");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  bool all_ok = true;
  // Kirim command ke joint Gazebo
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      // Default: velocity control (sesuai YAML dan Xacro)
      this->dataPtr->sim_joints_[j]->SetVelocity(0, this->dataPtr->joint_velocity_cmd_[j]);
    } else {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("GazeboSystem"), "Joint '%s' pointer NULL saat write!", this->dataPtr->joint_names_[j].c_str());
      publish_health_status("ERROR: Joint " + this->dataPtr->joint_names_[j] + " pointer NULL saat write");
      all_ok = false;
    }
  }
  if (all_ok) publish_health_status("OK: write success");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Default: tidak melakukan apa-apa, return OK
  return hardware_interface::return_type::OK;
}

}  // namespace gazebo_ros2_control

#include "pluginlib/class_list_macros.hpp" // Macro export pluginlib
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros2_control::GazeboSystem, hardware_interface::SystemInterface) // Export plugin ke ros2_control