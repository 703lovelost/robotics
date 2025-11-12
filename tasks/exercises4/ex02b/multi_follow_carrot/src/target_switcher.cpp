#include <memory>
#include <string>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TargetSwitcher : public rclcpp::Node {
public:
  TargetSwitcher() : Node("target_switcher") {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "turtle1");

    carrot_frame_  = this->declare_parameter<std::string>("carrot_frame",  "carrot");
    carrot2_frame_ = this->declare_parameter<std::string>("carrot2_frame", "carrot2");
    static_frame_  = this->declare_parameter<std::string>("static_frame",  "static_target");

    radius_ = this->declare_parameter<double>("radius", 2.0);
    omega_  = this->declare_parameter<double>("angular_speed", 0.6);
    dir_    = this->declare_parameter<int>("direction_of_rotation", 1);  // 1 по часовой, -1 против
    if (dir_ == 0) dir_ = 1;

    static_x_ = this->declare_parameter<double>("static_x", 8.0);
    static_y_ = this->declare_parameter<double>("static_y", 2.0);

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = now();
    angle_ = 0.0;

    timer_ = this->create_wall_timer(50ms, std::bind(&TargetSwitcher::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "Targets: (%s,%s) around '%s' R=%.3f dir=%d ω=%.3f; static '%s'=(%.2f, %.2f)",
      carrot_frame_.c_str(), carrot2_frame_.c_str(), parent_frame_.c_str(),
      radius_, dir_, omega_, static_frame_.c_str(), static_x_, static_y_);
  }

private:
  void onTimer() {
    rclcpp::Time tnow = now();
    double dt = (tnow - last_time_).seconds();
    last_time_ = tnow;

    angle_ += dir_ * omega_ * dt;  // общая фаза

    publishOrbit(tnow, parent_frame_, carrot_frame_, angle_);

    publishOrbit(tnow, parent_frame_, carrot2_frame_, angle_ + M_PI);

    geometry_msgs::msg::TransformStamped st;
    st.header.stamp = tnow;
    st.header.frame_id = "world";
    st.child_frame_id  = static_frame_;
    st.transform.translation.x = static_x_;
    st.transform.translation.y = static_y_;
    st.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0.0, 0.0, 0.0);
    st.transform.rotation.x = q.x();
    st.transform.rotation.y = q.y();
    st.transform.rotation.z = q.z();
    st.transform.rotation.w = q.w();
    broadcaster_->sendTransform(st);
  }

  void publishOrbit(const rclcpp::Time& tnow,
                    const std::string& parent,
                    const std::string& child,
                    double ang) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = tnow;
    tf.header.frame_id = parent;
    tf.child_frame_id  = child;

    tf.transform.translation.x = radius_ * std::cos(ang);
    tf.transform.translation.y = radius_ * std::sin(ang);
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q; q.setRPY(0.0, 0.0, ang);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    broadcaster_->sendTransform(tf);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  std::string parent_frame_, carrot_frame_, carrot2_frame_, static_frame_;
  double radius_{2.0}, omega_{0.6}, angle_{0.0};
  int dir_{1};
  double static_x_{8.0}, static_y_{2.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSwitcher>());
  rclcpp::shutdown();
  return 0;
}

