#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

class TimeTravelFollower : public rclcpp::Node {
public:
  TimeTravelFollower() : Node("turtle_time_travel_listener"),
                         buffer_(this->get_clock()),
                         listener_(buffer_) {
    delay_sec_ = this->declare_parameter<double>("delay", 1.0);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(16),
      std::bind(&TimeTravelFollower::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Follower started. delay=%.3f s", delay_sec_);
  }

private:
  void onTimer() {
    using namespace std::chrono_literals;
    const auto now = this->get_clock()->now();
    const auto past = now - rclcpp::Duration::from_seconds(delay_sec_);

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = buffer_.lookupTransform(
        "turtle2", now, "turtle1", past, "world", 50ms);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF lookup failed: %s", ex.what());
      return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;

    auto cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = 4.0 * std::atan2(y, x);
    cmd.linear.x  = 0.5 * std::sqrt(x * x + y * y);
    vel_pub_->publish(cmd);
  }

  double delay_sec_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeTravelFollower>());
  rclcpp::shutdown();
  return 0;
}

