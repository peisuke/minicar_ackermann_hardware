#ifndef MINICAR_ACKERMANN_HARDWARE__PCA9685_SERVO_CONTROLLER_HPP_
#define MINICAR_ACKERMANN_HARDWARE__PCA9685_SERVO_CONTROLLER_HPP_

#include <cstdint>
#include <string>

namespace minicar_ackermann_hardware
{

/**
 * @brief PCA9685 PWM controller for ESC and Servo control
 *
 * Handles 50Hz PWM signal generation for RC ESC and servo.
 * - ESC: 1.5ms = neutral, 1.0-1.5ms = reverse, 1.5-2.0ms = forward
 * - Servo: 1.5ms = center, 1.0-2.0ms = full range
 */
class PCA9685ServoController
{
public:
  // PWM channel assignments
  static constexpr int CHANNEL_ESC = 0;      // Throttle ESC
  static constexpr int CHANNEL_SERVO = 1;    // Steering servo

  // PWM timing (50Hz = 20ms period)
  static constexpr double PWM_FREQUENCY_HZ = 50.0;
  static constexpr double PWM_PERIOD_MS = 20.0;
  static constexpr int PCA9685_RESOLUTION = 4096;  // 12-bit resolution

  // Pulse width limits (milliseconds)
  static constexpr double PULSE_MIN_MS = 1.0;      // Minimum pulse width
  static constexpr double PULSE_CENTER_MS = 1.5;   // Center/neutral position
  static constexpr double PULSE_MAX_MS = 2.0;      // Maximum pulse width

  /**
   * @brief Constructor
   * @param i2c_device I2C device path (e.g., "/dev/i2c-1")
   * @param i2c_address PCA9685 I2C address (default: 0x40)
   */
  PCA9685ServoController(
    const std::string& i2c_device = "/dev/i2c-1",
    uint8_t i2c_address = 0x40);

  ~PCA9685ServoController();

  /**
   * @brief Initialize the PCA9685 controller
   * @return true if successful
   */
  bool initialize();

  /**
   * @brief Shutdown the controller
   */
  void shutdown();

  /**
   * @brief Set ESC throttle
   * @param throttle Throttle value: -1.0 (full reverse) to 1.0 (full forward), 0.0 = neutral
   */
  void set_throttle(double throttle);

  /**
   * @brief Set steering servo position
   * @param angle Steering angle: -1.0 (full left) to 1.0 (full right), 0.0 = center
   */
  void set_steering(double angle);

  /**
   * @brief Set all outputs to neutral/safe position
   */
  void set_neutral();

  /**
   * @brief Check if controller is initialized
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief Set servo pulse width limits (for calibration)
   * @param min_ms Minimum pulse width in milliseconds
   * @param center_ms Center pulse width in milliseconds
   * @param max_ms Maximum pulse width in milliseconds
   */
  void set_servo_limits(double min_ms, double center_ms, double max_ms);

  /**
   * @brief Set ESC pulse width limits (for calibration)
   * @param min_ms Minimum pulse width in milliseconds (full reverse)
   * @param center_ms Center pulse width in milliseconds (neutral)
   * @param max_ms Maximum pulse width in milliseconds (full forward)
   */
  void set_esc_limits(double min_ms, double center_ms, double max_ms);

  /**
   * @brief Set clock correction factor for PCA9685 oscillator drift
   * @param correction Correction factor (e.g., 1.1 for a fast oscillator)
   */
  void set_clock_correction(double correction);

private:
  /**
   * @brief Set PWM pulse width on a channel
   * @param channel PCA9685 channel (0-15)
   * @param pulse_ms Pulse width in milliseconds
   */
  void set_pwm_pulse(int channel, double pulse_ms);

  /**
   * @brief Convert pulse width to PCA9685 PWM value
   * @param pulse_ms Pulse width in milliseconds
   * @return PWM off value (0-4095)
   */
  uint16_t pulse_to_pwm(double pulse_ms) const;

  /**
   * @brief Write a byte to PCA9685 register
   */
  bool write_register(uint8_t reg, uint8_t value);

  /**
   * @brief Read a byte from PCA9685 register
   */
  uint8_t read_register(uint8_t reg);

  // I2C configuration
  std::string i2c_device_;
  uint8_t i2c_address_;
  int i2c_fd_;

  // State
  bool initialized_;

  // Servo calibration limits
  double servo_min_ms_;
  double servo_center_ms_;
  double servo_max_ms_;

  // ESC calibration limits
  double esc_min_ms_;
  double esc_center_ms_;
  double esc_max_ms_;

  // PCA9685 register addresses
  static constexpr uint8_t REG_MODE1 = 0x00;
  static constexpr uint8_t REG_MODE2 = 0x01;
  static constexpr uint8_t REG_PRESCALE = 0xFE;
  static constexpr uint8_t REG_LED0_ON_L = 0x06;
  static constexpr uint8_t REG_LED0_ON_H = 0x07;
  static constexpr uint8_t REG_LED0_OFF_L = 0x08;
  static constexpr uint8_t REG_LED0_OFF_H = 0x09;

  // PCA9685 mode bits
  static constexpr uint8_t MODE1_RESTART = 0x80;
  static constexpr uint8_t MODE1_SLEEP = 0x10;
  static constexpr uint8_t MODE1_AI = 0x20;      // Auto-increment

  // PCA9685 oscillator frequency
  static constexpr double OSC_FREQUENCY = 25000000.0;  // 25 MHz

  // Clock correction factor for PCA9685 oscillator drift
  // Adjusts pulse width calculation to compensate for individual board variance
  double clock_correction_ = 1.0;
};

}  // namespace minicar_ackermann_hardware

#endif  // MINICAR_ACKERMANN_HARDWARE__PCA9685_SERVO_CONTROLLER_HPP_
