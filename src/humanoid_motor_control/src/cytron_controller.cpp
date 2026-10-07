#include "humanoid_motor_control/cytron_controller.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cmath>

namespace humanoid_motor_control
{

CytronController::CytronController()
: rc1_pin_(-1), rc2_pin_(-1), is_initialized_(false), left_trim_(1.0), right_trim_(1.0)
{
}

CytronController::~CytronController()
{
  close();
}

bool CytronController::init(int rc1_pin, int rc2_pin)
{
  rc1_pin_ = rc1_pin;
  rc2_pin_ = rc2_pin;
  
  // Initialize WiringPi using BCM GPIO numbering
  // Connect to pigpio daemon
  if ((pigpio_handle_ = pigpio_start(NULL, NULL)) < 0) {
    std::cerr << "Error: Failed to start pigpio daemon interface" << std::endl;
    return false;
  }

  // Set pins as outputs and initialize servo pulsewidths via daemon
  set_mode(pigpio_handle_, rc1_pin_, PI_OUTPUT);
  set_mode(pigpio_handle_, rc2_pin_, PI_OUTPUT);

  // Initialize both channels to neutral position (1500μs)
  // Use daemon servo commands which run continuously
  std::cout << "Initializing Cytron MDDRC10 - sending neutral signals..." << std::endl;
  set_servo_pulsewidth(pigpio_handle_, rc1_pin_, PWM_NEUTRAL);
  set_servo_pulsewidth(pigpio_handle_, rc2_pin_, PWM_NEUTRAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  is_initialized_ = true;
  std::cout << "Cytron controller initialized on GPIO pins RC1=" 
            << rc1_pin_ << ", RC2=" << rc2_pin_ << std::endl;
  
  return true;
}

void CytronController::setTrim(double left_trim, double right_trim)
{
  left_trim_ = std::max(0.0, left_trim);
  right_trim_ = std::max(0.0, right_trim);
}

void CytronController::setMotorSpeed(uint8_t motor_id, int16_t speed)
{
  if (!is_initialized_ || motor_id > 3) {
    return;
  }

  // Motors 0,2 = left (RC1), Motors 1,3 = right (RC2)
  if (motor_id == 0 || motor_id == 2) {
    // Left motors - send to RC1
    int16_t trimmed_speed = clampSpeed(static_cast<int16_t>(std::round(speed * left_trim_)));
    int pwm = speedToPWM(trimmed_speed);
    sendPWM(rc1_pin_, pwm);
  } else {
    // Right motors - send to RC2
    int16_t trimmed_speed = clampSpeed(static_cast<int16_t>(std::round(speed * right_trim_)));
    int pwm = speedToPWM(trimmed_speed);
    sendPWM(rc2_pin_, pwm);
  }
}

void CytronController::setLeftRight(int16_t left_speed, int16_t right_speed)
{
  if (!is_initialized_) {
    return;
  }

  // Apply motor trim multipliers
  int16_t trimmed_left = clampSpeed(static_cast<int16_t>(std::round(left_speed * left_trim_)));
  int16_t trimmed_right = clampSpeed(static_cast<int16_t>(std::round(right_speed * right_trim_)));

  int left_pwm = speedToPWM(trimmed_left);
  int right_pwm = speedToPWM(trimmed_right);

  sendPWM(rc1_pin_, left_pwm);
  sendPWM(rc2_pin_, right_pwm);
}

void CytronController::setAllMotors(const std::vector<int16_t> & speeds)
{
  if (speeds.size() != 4) {
    std::cerr << "Error: Expected 4 motor speeds" << std::endl;
    return;
  }

  // Average left motors (0, 2) and right motors (1, 3)
  int16_t left_avg = (speeds[0] + speeds[2]) / 2; 
  int16_t right_avg = (speeds[1] + speeds[3]) / 2;

  setLeftRight(left_avg, right_avg);
}

void CytronController::emergencyStop()
{
  if (!is_initialized_) {
    return;
  }

  // Send neutral position to both channels
  sendPWM(rc1_pin_, PWM_NEUTRAL);
  sendPWM(rc2_pin_, PWM_NEUTRAL);
  
  std::cout << "Emergency stop - motors neutral" << std::endl;
}

void CytronController::close()
{
  if (is_initialized_) {
    emergencyStop();
    // Wait a bit to ensure signals are sent
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Stop pigpio daemon connection
    if (pigpio_handle_ >= 0) {
      // stop servo pulses
      set_servo_pulsewidth(pigpio_handle_, rc1_pin_, 0);
      set_servo_pulsewidth(pigpio_handle_, rc2_pin_, 0);
      pigpio_stop(pigpio_handle_);
      pigpio_handle_ = -1;
    }
    is_initialized_ = false;
  }
}

int CytronController::speedToPWM(int16_t speed)
{
  // speed: -100 to 100
  // PWM: 1000 to 2000 microseconds
  // 1500 = neutral (stop)
  // <1500 = counter-clockwise (reverse)
  // >1500 = clockwise (forward)

  // Clamp speed to valid range
  speed = clampSpeed(speed);

  // Convert: speed of 100 -> 2000μs, speed of -100 -> 1000μs, speed of 0 -> 1500μs
  // Formula: PWM = 1500 + (speed * 5)
  int pwm = PWM_NEUTRAL + (speed * PWM_RANGE / 100);

  // Ensure within bounds
  pwm = std::clamp(pwm, PWM_MIN, PWM_MAX);

  return pwm;
}

void CytronController::sendPWM(int pin, int pulse_width_us)
{
  // Generate RC PWM signal using bit-banging
  // Standard RC PWM: 50Hz (20ms period), pulse width 1000-2000μs
  // Use daemon servo command to set the pulsewidth (daemon manages the 50Hz updates)
  if (pigpio_handle_ >= 0) {
    set_servo_pulsewidth(pigpio_handle_, pin, pulse_width_us);
  }
}

int16_t CytronController::clampSpeed(int16_t speed)
{
  return std::clamp(speed, static_cast<int16_t>(-100), static_cast<int16_t>(100));
}

}  // namespace humanoid_motor_control