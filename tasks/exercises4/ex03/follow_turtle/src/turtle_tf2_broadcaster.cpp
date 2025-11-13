#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TurtleTf2Broadcaster : public rclcpp::Node {
public:
  TurtleTf2Broadcaster() : Node("turtle_tf2_broadcaster") {
    turtle_name_ = this->declare_parameter<std::string>("turtle", "turtle1");
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/" + turtle_name_ + "/pose", 10,
      std::bind(&TurtleTf2Broadcaster::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TF broadcaster started for %s", turtle_name_.c_str());
  }

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtle_name_;
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
  }

  std::string turtle_name_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleTf2Broadcaster>());
  rclcpp::shutdown();
  return 0;
}

