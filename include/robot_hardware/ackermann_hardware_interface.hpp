#ifndef MINICAR_ACKERMANN_HARDWARE__ACKERMANN_HARDWARE_INTERFACE_HPP_
#define MINICAR_ACKERMANN_HARDWARE__ACKERMANN_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot_hardware/pca9685_servo_controller.hpp"

namespace minicar_ackermann_hardware
{

/**
 * @brief Hardware interface for Ackermann steering robot
 *
 * Handles:
 * - Rear wheel velocity control via ESC
 * - Front wheel steering position control via servo
 * - Open-loop odometry (no encoders)
 */
class AckermannHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AckermannHardwareInterface)

  // Joint indices
  static constexpr size_t JOINT_REAR_LEFT_WHEEL = 0;
  static constexpr size_t JOINT_REAR_RIGHT_WHEEL = 1;
  static constexpr size_t JOINT_FRONT_LEFT_STEERING = 2;
  static constexpr size_t JOINT_FRONT_RIGHT_STEERING = 3;
  static constexpr size_t NUM_JOINTS = 4;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Hardware parameters
  std::string i2c_device_;
  uint8_t pca9685_address_;

  // Wheel parameters
  double wheel_radius_;
  double max_wheel_velocity_;  // rad/s

  // Steering parameters
  double max_steering_angle_;  // radians

  // Servo/ESC calibration
  double servo_min_pulse_ms_;
  double servo_center_pulse_ms_;
  double servo_max_pulse_ms_;
  double esc_min_pulse_ms_;
  double esc_center_pulse_ms_;
  double esc_max_pulse_ms_;

  // Open-loop odometry settings
  bool use_open_loop_odometry_;
  double open_loop_velocity_scale_;

  // Hardware controller
  std::unique_ptr<PCA9685ServoController> pca9685_controller_;

  // State and command storage
  // Positions (radians)
  std::vector<double> hw_positions_;
  // Velocities (rad/s)
  std::vector<double> hw_velocities_;
  // Commands: velocity for wheels, position for steering
  std::vector<double> hw_commands_;

  // Previous state for integration
  rclcpp::Time prev_time_;
  bool first_read_;

  /**
   * @brief Convert wheel velocity command (rad/s) to ESC throttle (-1.0 to 1.0)
   */
  double velocity_to_throttle(double velocity) const;

  /**
   * @brief Convert steering angle (radians) to servo position (-1.0 to 1.0)
   */
  double angle_to_servo_position(double angle) const;

  /**
   * @brief Initialize hardware controller
   */
  bool initialize_hardware();
};

}  // namespace minicar_ackermann_hardware

#endif  // MINICAR_ACKERMANN_HARDWARE__ACKERMANN_HARDWARE_INTERFACE_HPP_
