#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pigpio.h>
#include <vector>
#include <string>
#include <mutex>

// Global variables for interrupt callbacks
std::vector<int64_t> g_encoder_counts(4, 0);
std::mutex g_encoder_mutex;

// Interrupt callback functions (one for each encoder)
void encoder0_callback(int gpio, int level, uint32_t tick) {
  if (level == 1) {  // Rising edge
    std::lock_guard<std::mutex> lock(g_encoder_mutex);
    g_encoder_counts[0]++;
  }
}

void encoder1_callback(int gpio, int level, uint32_t tick) {
  if (level == 1) {
    std::lock_guard<std::mutex> lock(g_encoder_mutex);
    g_encoder_counts[1]++;
  }
}

void encoder2_callback(int gpio, int level, uint32_t tick) {
  if (level == 1) {
    std::lock_guard<std::mutex> lock(g_encoder_mutex);
    g_encoder_counts[2]++;
  }
}

void encoder3_callback(int gpio, int level, uint32_t tick) {
  if (level == 1) {
    std::lock_guard<std::mutex> lock(g_encoder_mutex);
    g_encoder_counts[3]++;
  }
}

class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode()
  : Node("encoder_node")
  {
    // Declare parameters
    this->declare_parameter("encoder_pins", std::vector<int64_t>{17, 18, 27, 22});
    this->declare_parameter("pulses_per_revolution", 20);
    this->declare_parameter("wheel_radius", 0.05);
    this->declare_parameter("publish_rate", 50.0);

    // Get parameters
    auto pin_params = this->get_parameter("encoder_pins").as_integer_array();
    encoder_pins_.assign(pin_params.begin(), pin_params.end());
    ppr_ = this->get_parameter("pulses_per_revolution").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    double pub_rate = this->get_parameter("publish_rate").as_double();

    // Initialize storage
    g_encoder_counts.resize(encoder_pins_.size(), 0);
    last_encoder_counts_.resize(encoder_pins_.size(), 0);
    velocities_.resize(encoder_pins_.size(), 0.0);

    // Initialize pigpio
    if (gpioInitialise() < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio");
      RCLCPP_ERROR(this->get_logger(), "Make sure pigpiod is running: sudo pigpiod");
      rclcpp::shutdown();
      return;
    }

    // Setup GPIO pins and interrupts for encoders
    if (encoder_pins_.size() >= 1 && encoder_pins_[0] >= 0) {
      gpioSetMode(encoder_pins_[0], PI_INPUT);
      gpioSetPullUpDown(encoder_pins_[0], PI_PUD_UP);
      gpioSetAlertFunc(encoder_pins_[0], encoder0_callback);
    }
    
    if (encoder_pins_.size() >= 2 && encoder_pins_[1] >= 0) {
      gpioSetMode(encoder_pins_[1], PI_INPUT);
      gpioSetPullUpDown(encoder_pins_[1], PI_PUD_UP);
      gpioSetAlertFunc(encoder_pins_[1], encoder1_callback);
    }
    
    if (encoder_pins_.size() >= 3 && encoder_pins_[2] >= 0) {
      gpioSetMode(encoder_pins_[2], PI_INPUT);
      gpioSetPullUpDown(encoder_pins_[2], PI_PUD_UP);
      gpioSetAlertFunc(encoder_pins_[2], encoder2_callback);
    }
    
    if (encoder_pins_.size() >= 4 && encoder_pins_[3] >= 0) {
      gpioSetMode(encoder_pins_[3], PI_INPUT);
      gpioSetPullUpDown(encoder_pins_[3], PI_PUD_UP);
      gpioSetAlertFunc(encoder_pins_[3], encoder3_callback);
    }

    // Create publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "wheel_velocities", 10);

    // Create timer for publishing
    auto timer_interval = std::chrono::duration<double>(1.0 / pub_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_interval),
      std::bind(&EncoderNode::publishEncoderData, this));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Encoder node started with %zu encoders on GPIO pins: %d, %d, %d, %d", 
                encoder_pins_.size(), encoder_pins_[0], encoder_pins_[1], 
                encoder_pins_[2], encoder_pins_[3]);
  }

  ~EncoderNode()
  {
    // Clean up GPIO
    gpioTerminate();
  }

private:
  void publishEncoderData()
  {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    
    if (dt <= 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(g_encoder_mutex);

    // Calculate velocities
    for (size_t i = 0; i < g_encoder_counts.size() && i < encoder_pins_.size(); ++i) {
      int64_t delta_counts = g_encoder_counts[i] - last_encoder_counts_[i];
      
      // Calculate angular velocity (rad/s)
      double revolutions = static_cast<double>(delta_counts) / ppr_;
      double angular_vel = (revolutions * 2.0 * M_PI) / dt;
      
      // Calculate linear velocity (m/s)
      velocities_[i] = angular_vel * wheel_radius_;
      
      last_encoder_counts_[i] = g_encoder_counts[i];
    }

    // Publish joint states
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = current_time;
    joint_msg.name = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    joint_msg.position.resize(4, 0.0);
    joint_msg.velocity = velocities_;

    joint_state_pub_->publish(joint_msg);

    // Publish raw velocities
    auto vel_msg = std_msgs::msg::Float64MultiArray();
    vel_msg.data = velocities_;
    velocity_pub_->publish(vel_msg);

    last_time_ = current_time;
  }

  std::vector<int> encoder_pins_;
  std::vector<int64_t> last_encoder_counts_;
  std::vector<double> velocities_;
  
  int ppr_;
  double wheel_radius_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncoderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}