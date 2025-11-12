#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

class TurtleFollower : public rclcpp::Node {
public:
  TurtleFollower() : Node("turtle_follower") {
    turtlename_   = this->declare_parameter<std::string>("turtlename", "turtle2");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "carrot");

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(turtlename_ + "/cmd_vel", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&TurtleFollower::onTimer, this));
    RCLCPP_INFO(this->get_logger(), "Follower started: %s -> %s", turtlename_.c_str(), target_frame_.c_str());
  }

private:
  void onTimer() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        turtlename_, target_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;

    // простейший P-контроллер
    double angle = std::atan2(y, x);
    double dist  = std::hypot(x, y);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = std::min(2.0, 1.5 * dist);   // ограничим «газ»
    cmd.angular.z = 4.0 * angle;                 // разворот к цели

    pub_->publish(cmd);
  }

  std::string turtlename_, target_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleFollower>());
  rclcpp::shutdown();
  return 0;
}

