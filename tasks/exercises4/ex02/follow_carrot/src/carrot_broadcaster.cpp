#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class CarrotBroadcaster : public rclcpp::Node {
public:
  CarrotBroadcaster() : Node("carrot_broadcaster") {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "turtle1");
    child_frame_  = this->declare_parameter<std::string>("child_frame", "carrot");

    radius_ = this->declare_parameter<double>("radius", 2.0);
    omega_  = this->declare_parameter<double>("angular_speed", 0.6);
    dir_    = this->declare_parameter<int>("direction_of_rotation", 1);
    if (dir_ == 0) dir_ = 1;

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    last_time_ = now();
    timer_ = this->create_wall_timer(50ms, std::bind(&CarrotBroadcaster::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "Carrot around '%s': radius=%.3f, dir=%d, omega=%.3f rad/s",
      parent_frame_.c_str(), radius_, dir_, omega_);
  }

private:
  void onTimer() {
    rclcpp::Time tnow = now();
    double dt = (tnow - last_time_).seconds();
    last_time_ = tnow;

    angle_ += dir_ * omega_ * dt;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = tnow;
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id  = child_frame_;

    tf.transform.translation.x = radius_ * std::cos(angle_);
    tf.transform.translation.y = radius_ * std::sin(angle_);
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle_);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    broadcaster_->sendTransform(tf);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  std::string parent_frame_, child_frame_;
  double radius_{2.0}, omega_{0.6}, angle_{0.0};
  int dir_{1};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarrotBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

