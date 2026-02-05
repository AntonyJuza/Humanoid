#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wiringPi.h>
#include <vector>
#include <string>

class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode()
  : Node("encoder_node")
  {
    // Declare parameters
    this->declare_parameter("encoder_pins", std::vector<int64_t>{17, 18, 27, 22});  // GPIO pins
    this->declare_parameter("pulses_per_revolution", 20);  // Adjust based on your encoder
    this->declare_parameter("wheel_radius", 0.05);  // meters
    this->declare_parameter("publish_rate", 50.0);  // Hz

    // Get parameters
    auto pin_params = this->get_parameter("encoder_pins").as_integer_array();
    encoder_pins_.assign(pin_params.begin(), pin_params.end());
    ppr_ = this->get_parameter("pulses_per_revolution").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    double pub_rate = this->get_parameter("publish_rate").as_double();

    // Initialize encoder counts
    encoder_counts_.resize(encoder_pins_.size(), 0);
    last_encoder_counts_.resize(encoder_pins_.size(), 0);
    velocities_.resize(encoder_pins_.size(), 0.0);

    // Initialize WiringPi
    if (wiringPiSetupGpio() == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize WiringPi");
      rclcpp::shutdown();
      return;
    }

    // Setup GPIO pins for encoders
    for (size_t i = 0; i < encoder_pins_.size(); ++i) {
      pinMode(encoder_pins_[i], INPUT);
      pullUpDnControl(encoder_pins_[i], PUD_UP);
      
      // Setup interrupt for encoder pulse counting
      // Note: This is simplified. For proper quadrature encoding, you'd need both A and B channels
      wiringPiISR(encoder_pins_[i], INT_EDGE_RISING, 
                  [this, i]() { this->encoderCallback(i); });
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

    RCLCPP_INFO(this->get_logger(), "Encoder node started with %zu encoders", 
                encoder_pins_.size());
  }

private:
  void encoderCallback(size_t encoder_id)
  {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    encoder_counts_[encoder_id]++;
  }

  void publishEncoderData()
  {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    
    if (dt <= 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(encoder_mutex_);

    // Calculate velocities
    for (size_t i = 0; i < encoder_counts_.size(); ++i) {
      int64_t delta_counts = encoder_counts_[i] - last_encoder_counts_[i];
      
      // Calculate angular velocity (rad/s)
      double revolutions = static_cast<double>(delta_counts) / ppr_;
      double angular_vel = (revolutions * 2.0 * M_PI) / dt;
      
      // Calculate linear velocity (m/s)
      velocities_[i] = angular_vel * wheel_radius_;
      
      last_encoder_counts_[i] = encoder_counts_[i];
    }

    // Publish joint states
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = current_time;
    joint_msg.name = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
    joint_msg.position.resize(4, 0.0);  // Position integration could be added
    joint_msg.velocity = velocities_;

    joint_state_pub_->publish(joint_msg);

    // Publish raw velocities
    auto vel_msg = std_msgs::msg::Float64MultiArray();
    vel_msg.data = velocities_;
    velocity_pub_->publish(vel_msg);

    last_time_ = current_time;
  }

  std::vector<int> encoder_pins_;
  std::vector<int64_t> encoder_counts_;
  std::vector<int64_t> last_encoder_counts_;
  std::vector<double> velocities_;
  
  int ppr_;
  double wheel_radius_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Time last_time_;
  std::mutex encoder_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncoderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}