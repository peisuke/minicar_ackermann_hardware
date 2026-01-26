#include "robot_hardware/ackermann_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace minicar_ackermann_hardware
{

hardware_interface::CallbackReturn AckermannHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse hardware parameters
  i2c_device_ = info_.hardware_parameters.count("i2c_device") ?
    info_.hardware_parameters.at("i2c_device") : "/dev/i2c-1";

  pca9685_address_ = info_.hardware_parameters.count("pca9685_address") ?
    static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("pca9685_address"))) : 0x40;

  wheel_radius_ = info_.hardware_parameters.count("wheel_radius") ?
    std::stod(info_.hardware_parameters.at("wheel_radius")) : 0.032;

  max_wheel_velocity_ = info_.hardware_parameters.count("max_wheel_velocity_rad_per_sec") ?
    std::stod(info_.hardware_parameters.at("max_wheel_velocity_rad_per_sec")) : 30.0;

  max_steering_angle_ = info_.hardware_parameters.count("max_steering_angle_rad") ?
    std::stod(info_.hardware_parameters.at("max_steering_angle_rad")) : 0.5236;  // 30 degrees

  // Servo calibration
  servo_min_pulse_ms_ = info_.hardware_parameters.count("servo_min_pulse_ms") ?
    std::stod(info_.hardware_parameters.at("servo_min_pulse_ms")) : 1.0;
  servo_center_pulse_ms_ = info_.hardware_parameters.count("servo_center_pulse_ms") ?
    std::stod(info_.hardware_parameters.at("servo_center_pulse_ms")) : 1.5;
  servo_max_pulse_ms_ = info_.hardware_parameters.count("servo_max_pulse_ms") ?
    std::stod(info_.hardware_parameters.at("servo_max_pulse_ms")) : 2.0;

  // ESC calibration
  esc_min_pulse_ms_ = info_.hardware_parameters.count("esc_min_pulse_ms") ?
    std::stod(info_.hardware_parameters.at("esc_min_pulse_ms")) : 1.0;
  esc_center_pulse_ms_ = info_.hardware_parameters.count("esc_center_pulse_ms") ?
    std::stod(info_.hardware_parameters.at("esc_center_pulse_ms")) : 1.5;
  esc_max_pulse_ms_ = info_.hardware_parameters.count("esc_max_pulse_ms") ?
    std::stod(info_.hardware_parameters.at("esc_max_pulse_ms")) : 2.0;

  // Open-loop odometry
  use_open_loop_odometry_ = info_.hardware_parameters.count("use_open_loop_odometry") ?
    (info_.hardware_parameters.at("use_open_loop_odometry") == "true") : true;
  open_loop_velocity_scale_ = info_.hardware_parameters.count("open_loop_velocity_scale") ?
    std::stod(info_.hardware_parameters.at("open_loop_velocity_scale")) : 1.0;

  // Verify joint configuration
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AckermannHardwareInterface"),
      "Expected %zu joints, got %zu", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage
  hw_positions_.resize(NUM_JOINTS, 0.0);
  hw_velocities_.resize(NUM_JOINTS, 0.0);
  hw_commands_.resize(NUM_JOINTS, 0.0);

  first_read_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Hardware interface initialized with %zu joints", NUM_JOINTS);
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "  I2C device: %s, address: 0x%02X", i2c_device_.c_str(), pca9685_address_);
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "  Wheel radius: %.4f m, max velocity: %.2f rad/s",
    wheel_radius_, max_wheel_velocity_);
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "  Max steering angle: %.2f rad (%.1f deg)",
    max_steering_angle_, max_steering_angle_ * 180.0 / M_PI);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Configuring hardware interface...");

  // Initialize PCA9685 controller
  if (!initialize_hardware()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AckermannHardwareInterface"),
      "Failed to initialize hardware controller");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset commands
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_commands_[i] = 0.0;
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Hardware interface configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Activating hardware interface...");

  // Set neutral position
  if (pca9685_controller_) {
    pca9685_controller_->set_neutral();
  }

  first_read_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Hardware interface activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Deactivating hardware interface...");

  // Stop motors and center steering
  if (pca9685_controller_) {
    pca9685_controller_->set_neutral();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Hardware interface deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Cleaning up hardware interface...");

  if (pca9685_controller_) {
    pca9685_controller_->shutdown();
    pca9685_controller_.reset();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AckermannHardwareInterface"),
    "Hardware interface cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AckermannHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AckermannHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Rear wheels: velocity command
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[JOINT_REAR_LEFT_WHEEL].name,
      hardware_interface::HW_IF_VELOCITY, &hw_commands_[JOINT_REAR_LEFT_WHEEL]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[JOINT_REAR_RIGHT_WHEEL].name,
      hardware_interface::HW_IF_VELOCITY, &hw_commands_[JOINT_REAR_RIGHT_WHEEL]));

  // Front steering: position command
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[JOINT_FRONT_LEFT_STEERING].name,
      hardware_interface::HW_IF_POSITION, &hw_commands_[JOINT_FRONT_LEFT_STEERING]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[JOINT_FRONT_RIGHT_STEERING].name,
      hardware_interface::HW_IF_POSITION, &hw_commands_[JOINT_FRONT_RIGHT_STEERING]));

  return command_interfaces;
}

hardware_interface::return_type AckermannHardwareInterface::read(
  const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  if (first_read_) {
    prev_time_ = time;
    first_read_ = false;
    return hardware_interface::return_type::OK;
  }

  double dt = (time - prev_time_).seconds();
  prev_time_ = time;

  if (dt <= 0.0 || dt > 1.0) {
    return hardware_interface::return_type::OK;
  }

  if (use_open_loop_odometry_) {
    // Open-loop: estimate state from commands
    // Rear wheels: velocity from command
    hw_velocities_[JOINT_REAR_LEFT_WHEEL] =
      hw_commands_[JOINT_REAR_LEFT_WHEEL] * open_loop_velocity_scale_;
    hw_velocities_[JOINT_REAR_RIGHT_WHEEL] =
      hw_commands_[JOINT_REAR_RIGHT_WHEEL] * open_loop_velocity_scale_;

    // Integrate position for rear wheels
    hw_positions_[JOINT_REAR_LEFT_WHEEL] +=
      hw_velocities_[JOINT_REAR_LEFT_WHEEL] * dt;
    hw_positions_[JOINT_REAR_RIGHT_WHEEL] +=
      hw_velocities_[JOINT_REAR_RIGHT_WHEEL] * dt;

    // Front steering: position from command (instant)
    hw_positions_[JOINT_FRONT_LEFT_STEERING] = hw_commands_[JOINT_FRONT_LEFT_STEERING];
    hw_positions_[JOINT_FRONT_RIGHT_STEERING] = hw_commands_[JOINT_FRONT_RIGHT_STEERING];
    hw_velocities_[JOINT_FRONT_LEFT_STEERING] = 0.0;
    hw_velocities_[JOINT_FRONT_RIGHT_STEERING] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannHardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!pca9685_controller_ || !pca9685_controller_->is_initialized()) {
    return hardware_interface::return_type::ERROR;
  }

  // Average rear wheel velocity for ESC command
  double avg_wheel_velocity = (hw_commands_[JOINT_REAR_LEFT_WHEEL] +
                               hw_commands_[JOINT_REAR_RIGHT_WHEEL]) / 2.0;

  // Convert to throttle and send to ESC
  double throttle = velocity_to_throttle(avg_wheel_velocity);
  pca9685_controller_->set_throttle(throttle);

  // Average steering angle for servo command
  // (In Ackermann, left and right steering angles are slightly different,
  //  but we use average for simplicity with a single servo)
  double avg_steering_angle = (hw_commands_[JOINT_FRONT_LEFT_STEERING] +
                               hw_commands_[JOINT_FRONT_RIGHT_STEERING]) / 2.0;

  // Convert to servo position and send
  double servo_pos = angle_to_servo_position(avg_steering_angle);
  pca9685_controller_->set_steering(servo_pos);

  return hardware_interface::return_type::OK;
}

double AckermannHardwareInterface::velocity_to_throttle(double velocity) const
{
  // Convert wheel velocity (rad/s) to throttle (-1.0 to 1.0)
  if (std::abs(velocity) < 0.01) {
    return 0.0;
  }

  double normalized = velocity / max_wheel_velocity_;
  return std::max(-1.0, std::min(1.0, normalized));
}

double AckermannHardwareInterface::angle_to_servo_position(double angle) const
{
  // Convert steering angle (radians) to servo position (-1.0 to 1.0)
  if (std::abs(angle) < 0.001) {
    return 0.0;
  }

  double normalized = angle / max_steering_angle_;
  return std::max(-1.0, std::min(1.0, normalized));
}

bool AckermannHardwareInterface::initialize_hardware()
{
  pca9685_controller_ = std::make_unique<PCA9685ServoController>(
    i2c_device_, pca9685_address_);

  // Set calibration limits
  pca9685_controller_->set_servo_limits(
    servo_min_pulse_ms_, servo_center_pulse_ms_, servo_max_pulse_ms_);
  pca9685_controller_->set_esc_limits(
    esc_min_pulse_ms_, esc_center_pulse_ms_, esc_max_pulse_ms_);

  return pca9685_controller_->initialize();
}

}  // namespace minicar_ackermann_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  minicar_ackermann_hardware::AckermannHardwareInterface,
  hardware_interface::SystemInterface)
