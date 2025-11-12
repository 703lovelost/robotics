#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TurtleTFBroadcaster : public rclcpp::Node {
public:
  TurtleTFBroadcaster() : Node("turtle_tf2_broadcaster") {
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle1");
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    sub_ = this->create_subscription<turtlesim::msg::Pose>(
      turtlename_ + "/pose", 10,
      std::bind(&TurtleTFBroadcaster::poseCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "TF broadcaster started for %s", turtlename_.c_str());
  }

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_;
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    broadcaster_->sendTransform(t);
  }

  std::string turtlename_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

