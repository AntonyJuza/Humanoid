#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode()
  : Node("navigation_node"), e_stop_active_(false), fire_detected_(false), fire_dir_("center")
  {
    // Declare parameters
    this->declare_parameter("danger_dist_cm", 25.0);
    this->declare_parameter("caution_dist_cm", 50.0);
    this->declare_parameter("forward_speed", 0.25);
    this->declare_parameter("turn_speed", 0.6);

    danger_dist_ = this->get_parameter("danger_dist_cm").as_double();
    caution_dist_ = this->get_parameter("caution_dist_cm").as_double();
    forward_speed_ = this->get_parameter("forward_speed").as_double();
    turn_speed_ = this->get_parameter("turn_speed").as_double();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);

    distances_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/ultrasonic/distances", 10, std::bind(&NavigationNode::distances_callback, this, _1));
    
    fire_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fire_detected", 10, std::bind(&NavigationNode::fire_callback, this, _1));
      
    fire_dir_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fire_direction", 10, std::bind(&NavigationNode::fire_dir_callback, this, _1));
      
    e_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", 10, std::bind(&NavigationNode::estop_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Navigation Node initialized");
  }

private:
  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    e_stop_active_ = msg->data;
    if (e_stop_active_) {
      publishCmdVel(0.0, 0.0);
      publishStatus("E-STOP ACTIVE");
    }
  }

  void fire_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    fire_detected_ = msg->data;
  }

  void fire_dir_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    fire_dir_ = msg->data;
  }

  void publishCmdVel(double linear, double angular)
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_pub_->publish(msg);
  }

  void publishStatus(const std::string& status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "%s", status.c_str());
  }

  void distances_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (e_stop_active_) {
      return; 
    }

    if (msg->data.size() < 3) return;

    float front = msg->data[0];
    float fl = msg->data[1];
    float fr = msg->data[2];

    char status_buf[256];

    // Obstacle avoidance logic overriding everything else
    if (front < danger_dist_ || fl < danger_dist_ || fr < danger_dist_) {
      // Very close! Stop and spin away from the closest danger.
      publishCmdVel(0.0, (fl < fr) ? -turn_speed_ : turn_speed_); // spin right if left is closer, else spin left
      
      snprintf(status_buf, sizeof(status_buf), "%s - BLOCKED F:%.1f FL:%.1f FR:%.1f - TURNING %s", 
               (fire_detected_ ? "APPROACH" : "SEARCH"), front, fl, fr, (fl < fr) ? "RIGHT" : "LEFT");
      publishStatus(status_buf);
      return;
    }

    if (fire_detected_) {
      // APPROACH MODE
      double lin = forward_speed_;
      double ang = 0.0;
      
      if (fire_dir_ == "left") {
          ang = turn_speed_ * 0.7; // Gentle turn left
      } else if (fire_dir_ == "right") {
          ang = -turn_speed_ * 0.7; // Gentle turn right
      }

      publishCmdVel(lin, ang);
      snprintf(status_buf, sizeof(status_buf), "APPROACH -> fire %s, F:%.1f spd=%.2f", 
               fire_dir_.c_str(), front, lin);
      publishStatus(status_buf);
    } else {
      // SEARCH MODE
      if (front < caution_dist_) {
        // Caution turn
        publishCmdVel(0.0, (fl < fr) ? -turn_speed_ : turn_speed_);
        snprintf(status_buf, sizeof(status_buf), "SEARCH - front %.1fcm, turning %s", 
                 front, (fl < fr) ? "RIGHT" : "LEFT");
      } else if (fl < caution_dist_) {
        publishCmdVel(forward_speed_ * 0.5, -turn_speed_ * 0.5); // curve right
        snprintf(status_buf, sizeof(status_buf), "SEARCH - curve RIGHT");
      } else if (fr < caution_dist_) {
        publishCmdVel(forward_speed_ * 0.5, turn_speed_ * 0.5); // curve left
        snprintf(status_buf, sizeof(status_buf), "SEARCH - curve LEFT");
      } else {
        publishCmdVel(forward_speed_, 0.0);
        snprintf(status_buf, sizeof(status_buf), "SEARCH - moving forward F:%.1f FL:%.1f FR:%.1f", front, fl, fr);
      }
      publishStatus(status_buf);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr distances_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fire_dir_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;

  double danger_dist_;
  double caution_dist_;
  double forward_speed_;
  double turn_speed_;

  bool e_stop_active_;
  bool fire_detected_;
  std::string fire_dir_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}
