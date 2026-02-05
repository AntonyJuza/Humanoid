#include "humanoid_motor_control/cytron_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using humanoid_motor_control::CytronController;

class CytronNode : public rclcpp::Node
{
public:
    CytronNode()
    : Node("cytron_controller_node")
    {
        // Parameters for motor pins
        int rc1 = this->declare_parameter("rc1_pin", 12);
        int rc2 = this->declare_parameter("rc2_pin", 13);

        if (!controller_.init(rc1, rc2)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CytronController");
        } else {
            RCLCPP_INFO(this->get_logger(), "CytronController initialized on RC1=%d RC2=%d", rc1, rc2);
        }

        // Publisher for wheel velocities
        wheel_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/wheel_velocities", 10);

        // Subscriber for /cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CytronNode::cmdVelCallback, this, std::placeholders::_1));

        // Maximum linear speed (m/s) and angular speed (rad/s) for mapping
        max_linear_speed_ = this->declare_parameter("max_linear_speed", 0.5);
        max_angular_speed_ = this->declare_parameter("max_angular_speed", 1.0);
        wheel_base_ = this->declare_parameter("wheel_base", 0.15); // meters
    }

    ~CytronNode()
    {
        controller_.close();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Map linear and angular velocities to left/right motor speeds (-100 to 100)
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        int left_speed = static_cast<int>(100 * (linear - angular * wheel_base_ / 2.0) / max_linear_speed_);
        int right_speed = static_cast<int>(100 * (linear + angular * wheel_base_ / 2.0) / max_linear_speed_);

        // Clamp speeds to [-100, 100]
        left_speed = std::clamp(left_speed, -100, 100);
        right_speed = std::clamp(right_speed, -100, 100);

        // Send speeds to CytronController
        controller_.setLeftRight(left_speed, right_speed);

        // Publish wheel velocities for monitoring
        auto wheel_msg = geometry_msgs::msg::Twist();
        wheel_msg.linear.x = static_cast<double>(left_speed);
        wheel_msg.linear.y = static_cast<double>(right_speed);
        wheel_vel_pub_->publish(wheel_msg);
    }

    CytronController controller_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_pub_;

    double max_linear_speed_;
    double max_angular_speed_;
    double wheel_base_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CytronNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
