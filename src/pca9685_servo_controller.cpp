#include "robot_hardware/pca9685_servo_controller.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

namespace minicar_ackermann_hardware
{

PCA9685ServoController::PCA9685ServoController(
  const std::string& i2c_device,
  uint8_t i2c_address)
: i2c_device_(i2c_device)
, i2c_address_(i2c_address)
, i2c_fd_(-1)
, initialized_(false)
, servo_min_ms_(PULSE_MIN_MS)
, servo_center_ms_(PULSE_CENTER_MS)
, servo_max_ms_(PULSE_MAX_MS)
, esc_min_ms_(PULSE_MIN_MS)
, esc_center_ms_(PULSE_CENTER_MS)
, esc_max_ms_(PULSE_MAX_MS)
{
}

PCA9685ServoController::~PCA9685ServoController()
{
  shutdown();
}

bool PCA9685ServoController::initialize()
{
  if (initialized_) {
    return true;
  }

  // Open I2C device
  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    std::cerr << "Failed to open I2C device: " << i2c_device_
              << " - " << strerror(errno) << std::endl;
    return false;
  }

  // Set I2C slave address
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    std::cerr << "Failed to set I2C address: 0x" << std::hex
              << static_cast<int>(i2c_address_) << std::dec
              << " - " << strerror(errno) << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Reset PCA9685
  write_register(REG_MODE1, MODE1_SLEEP);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Calculate prescale value for 50Hz
  // prescale = round(OSC_FREQ / (4096 * freq)) - 1
  double prescale_val = std::round(OSC_FREQUENCY / (PCA9685_RESOLUTION * PWM_FREQUENCY_HZ)) - 1;
  uint8_t prescale = static_cast<uint8_t>(prescale_val);

  std::cout << "PCA9685: Setting prescale to " << static_cast<int>(prescale)
            << " for " << PWM_FREQUENCY_HZ << " Hz" << std::endl;

  // Set prescale (must be in sleep mode)
  write_register(REG_PRESCALE, prescale);

  // Wake up and enable auto-increment
  write_register(REG_MODE1, MODE1_AI);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Restart
  uint8_t mode1 = read_register(REG_MODE1);
  write_register(REG_MODE1, mode1 | MODE1_RESTART);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Set initial neutral position
  set_neutral();

  initialized_ = true;
  std::cout << "PCA9685 Servo Controller initialized successfully" << std::endl;
  std::cout << "  I2C device: " << i2c_device_ << std::endl;
  std::cout << "  I2C address: 0x" << std::hex << static_cast<int>(i2c_address_)
            << std::dec << std::endl;
  std::cout << "  PWM frequency: " << PWM_FREQUENCY_HZ << " Hz" << std::endl;

  return true;
}

void PCA9685ServoController::shutdown()
{
  if (initialized_) {
    // Set neutral before shutdown
    set_neutral();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }

  initialized_ = false;
}

void PCA9685ServoController::set_throttle(double throttle)
{
  if (!initialized_) {
    return;
  }

  // Clamp throttle to [-1.0, 1.0]
  throttle = std::max(-1.0, std::min(1.0, throttle));

  // Convert to pulse width
  double pulse_ms;
  if (throttle >= 0) {
    // Forward: center to max
    pulse_ms = esc_center_ms_ + throttle * (esc_max_ms_ - esc_center_ms_);
  } else {
    // Reverse: center to min
    pulse_ms = esc_center_ms_ + throttle * (esc_center_ms_ - esc_min_ms_);
  }

  set_pwm_pulse(CHANNEL_ESC, pulse_ms);
}

void PCA9685ServoController::set_steering(double angle)
{
  if (!initialized_) {
    return;
  }

  // Clamp angle to [-1.0, 1.0]
  angle = std::max(-1.0, std::min(1.0, angle));

  // Convert to pulse width
  double pulse_ms;
  if (angle >= 0) {
    // Right: center to max
    pulse_ms = servo_center_ms_ + angle * (servo_max_ms_ - servo_center_ms_);
  } else {
    // Left: center to min
    pulse_ms = servo_center_ms_ + angle * (servo_center_ms_ - servo_min_ms_);
  }

  set_pwm_pulse(CHANNEL_SERVO, pulse_ms);
}

void PCA9685ServoController::set_neutral()
{
  set_pwm_pulse(CHANNEL_ESC, esc_center_ms_);
  set_pwm_pulse(CHANNEL_SERVO, servo_center_ms_);
}

void PCA9685ServoController::set_servo_limits(double min_ms, double center_ms, double max_ms)
{
  servo_min_ms_ = min_ms;
  servo_center_ms_ = center_ms;
  servo_max_ms_ = max_ms;
}

void PCA9685ServoController::set_esc_limits(double min_ms, double center_ms, double max_ms)
{
  esc_min_ms_ = min_ms;
  esc_center_ms_ = center_ms;
  esc_max_ms_ = max_ms;
}

void PCA9685ServoController::set_pwm_pulse(int channel, double pulse_ms)
{
  if (i2c_fd_ < 0 || channel < 0 || channel > 15) {
    return;
  }

  uint16_t off_value = pulse_to_pwm(pulse_ms);

  // Calculate register addresses for this channel
  uint8_t reg_base = REG_LED0_ON_L + (channel * 4);

  // ON time = 0 (start of period)
  // OFF time = calculated value
  uint8_t data[4] = {
    0x00,                               // ON_L
    0x00,                               // ON_H
    static_cast<uint8_t>(off_value & 0xFF),       // OFF_L
    static_cast<uint8_t>((off_value >> 8) & 0x0F) // OFF_H
  };

  // Write all 4 registers (auto-increment enabled)
  uint8_t buffer[5] = {reg_base, data[0], data[1], data[2], data[3]};
  if (write(i2c_fd_, buffer, 5) != 5) {
    std::cerr << "Failed to write PWM value to channel " << channel << std::endl;
  }
}

uint16_t PCA9685ServoController::pulse_to_pwm(double pulse_ms) const
{
  // Convert pulse width to PWM value
  // PWM value = (pulse_ms / period_ms) * 4096
  double duty = pulse_ms / PWM_PERIOD_MS;
  uint16_t pwm_value = static_cast<uint16_t>(duty * PCA9685_RESOLUTION);

  // Clamp to valid range
  return std::min(static_cast<uint16_t>(4095), pwm_value);
}

bool PCA9685ServoController::write_register(uint8_t reg, uint8_t value)
{
  if (i2c_fd_ < 0) {
    return false;
  }

  uint8_t buffer[2] = {reg, value};
  return write(i2c_fd_, buffer, 2) == 2;
}

uint8_t PCA9685ServoController::read_register(uint8_t reg)
{
  if (i2c_fd_ < 0) {
    return 0;
  }

  if (write(i2c_fd_, &reg, 1) != 1) {
    return 0;
  }

  uint8_t value = 0;
  if (read(i2c_fd_, &value, 1) != 1) {
    return 0;
  }

  return value;
}

}  // namespace minicar_ackermann_hardware
