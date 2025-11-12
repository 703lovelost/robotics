#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

static inline double wrapAngle(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

template <typename T>
static inline T clamp(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}

class MoveToGoal : public rclcpp::Node {
public:
  MoveToGoal(double goal_x, double goal_y, double goal_theta)
  : Node("move_to_goal"),
    goal_x_(goal_x), goal_y_(goal_y), goal_theta_(goal_theta)
  {
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    sub_pose_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&MoveToGoal::poseCallback, this, std::placeholders::_1)
    );

    timer_ = create_wall_timer(50ms, std::bind(&MoveToGoal::controlStep, this));

    RCLCPP_INFO(get_logger(),
      "Target: x=%.3f y=%.3f theta=%.3f",
      goal_x_, goal_y_, goal_theta_);

    done_future_ = done_promise_.get_future().share();
  }

  std::shared_future<void> done_future() const {
    return done_future_;
  }

private:
  enum class Stage { GO_TO_POINT, ALIGN_FINAL, DONE };

  std::promise<void> done_promise_;
  std::shared_future<void> done_future_;
  bool finished_{false};

  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    pose_ = *msg;
    have_pose_ = true;
  }

  void controlStep() {
    if (!have_pose_) return;

    geometry_msgs::msg::Twist cmd;

    const double k_lin = 1.5;
    const double k_ang = 6.0;
    const double k_align = 4.0;

    const double v_max = 2.0;
    const double w_max = 2.0;

    const double psi_ok = 0.25;
    const double eps_pos = 0.05;
    const double eps_theta = 0.02;

    switch (stage_) {
      case Stage::GO_TO_POINT: {
        double dx = goal_x_ - pose_.x;
        double dy = goal_y_ - pose_.y;
        double dist = std::hypot(dx, dy);

        if (dist <= eps_pos) {
          cmd.linear.x = 0.0; cmd.angular.z = 0.0;
          pub_cmd_->publish(cmd);
          stage_ = Stage::ALIGN_FINAL;
          RCLCPP_INFO(get_logger(), "Reached (dist=%.3f). Switching to trajectory alignment.", dist);
          break;
        }

        double psi = std::atan2(dy, dx);
        double e_psi = wrapAngle(psi - pose_.theta);

        double w = clamp(k_ang * e_psi, -w_max, w_max);
        double v = 0.0;

        if (std::fabs(e_psi) < psi_ok) {
          v = clamp(k_lin * dist, 0.0, v_max);
          v *= std::cos(e_psi);
          if (v < 0.0) v = 0.0;
        }

        cmd.linear.x = v;
        cmd.angular.z = w;
        pub_cmd_->publish(cmd);
        break;
      }

      case Stage::ALIGN_FINAL: {
        double e_yaw = wrapAngle(goal_theta_ - pose_.theta);

        if (std::fabs(e_yaw) <= eps_theta) {
          cmd.linear.x = 0.0; cmd.angular.z = 0.0;
          pub_cmd_->publish(cmd);
          stage_ = Stage::DONE;

          RCLCPP_INFO(get_logger(), "Trajectory alignment has been done.");
          break;
        }

        cmd.linear.x = 0.0;
        cmd.angular.z = clamp(k_align * e_yaw, -w_max, w_max);
        pub_cmd_->publish(cmd);
        break;
      }

      case Stage::DONE:
      default:
        cmd.linear.x = 0.0; cmd.angular.z = 0.0;
        pub_cmd_->publish(cmd);

        if (timer_) {
            timer_->cancel();
        }

        if (!finished_) {
          finished_ = true;
          done_promise_.set_value();
          RCLCPP_INFO(get_logger(), "Ending movement.");
        }

        break;
    }
  }

  double goal_x_{0.0}, goal_y_{0.0}, goal_theta_{0.0};

  turtlesim::msg::Pose pose_{};
  bool have_pose_{false};
  Stage stage_{Stage::GO_TO_POINT};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 4) {
    RCLCPP_ERROR(rclcpp::get_logger("move_to_goal"),
                 "Usage: ros2 run move_to_goal move_to_goal -- x y theta (рад)");
    return 1;
  }

  try {
    double x = std::stod(argv[1]);
    double y = std::stod(argv[2]);
    double th = std::stod(argv[3]);

    auto node = std::make_shared<MoveToGoal>(x, y, th);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    auto rc = exec.spin_until_future_complete(node->done_future());
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("move_to_goal"), "Reached the destination point, exiting the spin.");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("move_to_goal"), "Exiting spin before reaching the destination: %d", static_cast<int>(rc));
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("move_to_goal"), "Argument parsing error: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  
  rclcpp::shutdown();
  return 0;
}
