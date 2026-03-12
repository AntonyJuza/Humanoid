#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <pigpiod_if2.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

// Global pigpio handle
int pigpio_handle = -1;

struct Sensor {
    std::string name;
    int trig;
    int echo;
    uint32_t startTick;
    float distance;
    bool newReading;
    std::vector<float> history;
};

// Global array for callbacks
Sensor* g_sensors_ptr = nullptr;
size_t g_num_sensors = 0;

void echoCallback(int pi, unsigned user_gpio, unsigned edge, uint32_t tick) {
    if (!g_sensors_ptr) return;
    
    for (size_t i = 0; i < g_num_sensors; ++i) {
        if (g_sensors_ptr[i].echo == (int)user_gpio) {
            if (edge == RISING_EDGE) { // Rising edge (start of pulse)
                g_sensors_ptr[i].startTick = tick;
            } else if (edge == FALLING_EDGE) { // Falling edge (end of pulse)
                uint32_t diff = tick - g_sensors_ptr[i].startTick;
                float dist = (diff * 0.0343) / 2.0;
                // Cap the reading at 200 cm realistically
                if (dist > 200.0) dist = 200.0;
                
                g_sensors_ptr[i].distance = dist;
                g_sensors_ptr[i].newReading = true;
            }
            break;
        }
    }
}

class UltrasonicSensorNode : public rclcpp::Node
{
public:
  UltrasonicSensorNode()
  : Node("ultrasonic_sensor_node")
  {
    // Declare parameters
    this->declare_parameter("front_trig", 5);
    this->declare_parameter("front_echo", 6);
    this->declare_parameter("front_left_trig", 17);
    this->declare_parameter("front_left_echo", 27);
    this->declare_parameter("front_right_trig", 22);
    this->declare_parameter("front_right_echo", 23);
    this->declare_parameter("publish_rate_hz", 10.0);
    this->declare_parameter("moving_avg_size", 5);

    int front_trig = this->get_parameter("front_trig").as_int();
    int front_echo = this->get_parameter("front_echo").as_int();
    int front_left_trig = this->get_parameter("front_left_trig").as_int();
    int front_left_echo = this->get_parameter("front_left_echo").as_int();
    int front_right_trig = this->get_parameter("front_right_trig").as_int();
    int front_right_echo = this->get_parameter("front_right_echo").as_int();
    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    moving_avg_size_ = this->get_parameter("moving_avg_size").as_int();

    sensors_ = {
      {"front", front_trig, front_echo, 0, 200.0, false, {}},
      {"front_left", front_left_trig, front_left_echo, 0, 200.0, false, {}},
      {"front_right", front_right_trig, front_right_echo, 0, 200.0, false, {}}
    };

    g_sensors_ptr = sensors_.data();
    g_num_sensors = sensors_.size();

    // Connect to pigpiod daemon
    pigpio_handle = pigpio_start(NULL, NULL);
    if (pigpio_handle < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon. Make sure pigpiod is running: sudo pigpiod");
      rclcpp::shutdown();
      return;
    }

    for (auto& s : sensors_) {
      set_mode(pigpio_handle, s.trig, PI_OUTPUT);
      gpio_write(pigpio_handle, s.trig, 0);
      set_mode(pigpio_handle, s.echo, PI_INPUT);
      callback(pigpio_handle, s.echo, EITHER_EDGE, echoCallback);
    }
    
    // Give sensors a moment to settle
    rclcpp::sleep_for(500ms);

    distances_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/ultrasonic/distances", 10);
    front_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ultrasonic/front", 10);
    front_left_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ultrasonic/front_left", 10);
    front_right_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ultrasonic/front_right", 10);

    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period), 
      std::bind(&UltrasonicSensorNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Ultrasonic Sensor Node initialized at %.1f Hz", publish_rate_hz);
  }

  ~UltrasonicSensorNode() {
    if (pigpio_handle >= 0) {
      pigpio_stop(pigpio_handle);
    }
  }

private:
  void triggerSensor(int trigPin) {
    gpio_write(pigpio_handle, trigPin, 1);
    time_sleep(10e-6); // 10 microseconds pulse
    gpio_write(pigpio_handle, trigPin, 0);
  }

  float getSmoothedDistance(Sensor& s) {
    if (s.newReading) {
      s.history.push_back(s.distance);
      if (s.history.size() > moving_avg_size_) {
        s.history.erase(s.history.begin());
      }
      s.newReading = false;
    } else {
      // If we miss a reading, assume max distance limit
      s.history.push_back(200.0);
      if (s.history.size() > moving_avg_size_) {
        s.history.erase(s.history.begin());
      }
    }
    
    float sum = 0;
    for (float v : s.history) sum += v;
    return s.history.empty() ? 200.0 : (sum / s.history.size());
  }

  void timer_callback()
  {
    // Trigger sequentially to avoid cross-talk
    for (auto& s : sensors_) {
      triggerSensor(s.trig);
      time_sleep(15e-3); // 15ms delay
    }

    // Wait slightly for echoes
    time_sleep(10e-3);

    std_msgs::msg::Float32MultiArray array_msg;
    
    float front_dist = getSmoothedDistance(sensors_[0]);
    float front_left_dist = getSmoothedDistance(sensors_[1]);
    float front_right_dist = getSmoothedDistance(sensors_[2]);

    array_msg.data = {front_dist, front_left_dist, front_right_dist};
    distances_pub_->publish(array_msg);

    auto msg_f = std_msgs::msg::Float32(); msg_f.data = front_dist;
    auto msg_fl = std_msgs::msg::Float32(); msg_fl.data = front_left_dist;
    auto msg_fr = std_msgs::msg::Float32(); msg_fr.data = front_right_dist;

    front_pub_->publish(msg_f);
    front_left_pub_->publish(msg_fl);
    front_right_pub_->publish(msg_fr);
  }

  std::vector<Sensor> sensors_;
  size_t moving_avg_size_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr distances_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_right_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
  rclcpp::shutdown();
  return 0;
}
