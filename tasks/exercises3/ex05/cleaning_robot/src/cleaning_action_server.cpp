#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#include "cleaning_robot/action/cleaning_task.hpp"

using namespace std::chrono_literals;
using cleaning_robot::action::CleaningTask;

static inline double wrap(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

class CleaningActionServer : public rclcpp::Node {
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<CleaningTask>;

  CleaningActionServer()
  : Node("cleaning_action_server")
  {
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    sub_pose_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      [this](const turtlesim::msg::Pose & msg){
        std::lock_guard<std::mutex> lk(pose_mtx_);
        have_pose_ = true;
        
	if (has_prev_) {
          total_distance_runtime_ += std::hypot(msg.x - prev_pose_.x, msg.y - prev_pose_.y);
        }
        prev_pose_ = msg;
        has_prev_ = true;
        pose_ = msg;
      });

    action_server_ = rclcpp_action::create_server<CleaningTask>(
      this,
      "cleaning_task",
      std::bind(&CleaningActionServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CleaningActionServer::on_cancel, this, std::placeholders::_1),
      std::bind(&CleaningActionServer::on_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "cleaning_action_server: waiting for action 'cleaning_task'.");
  }

private:
  // ---- Action callbacks ----
  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID&,
                                      std::shared_ptr<const CleaningTask::Goal> goal)
  {
    if (!have_pose_) {
      RCLCPP_WARN(get_logger(), "Unknown turtle pose. Goal is cancelled.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->task_type != "clean_square" &&
        goal->task_type != "return_home" &&
        goal->task_type != "clean_circle")
    {
      RCLCPP_WARN(get_logger(), "Unknown task_type='%s'", goal->task_type.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->task_type == "clean_square" && goal->area_size <= 0.0) {
      RCLCPP_WARN(get_logger(), "area_size should be > 0 for clean_square");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
  {
    RCLCPP_INFO(get_logger(), "Cancel is called — stopping current move_to_goal.");
    stop_external_mtg();
    send_zero_twist();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread(&CleaningActionServer::execute, this, gh).detach();
  }

  // Исполнение цели
  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    auto goal = gh->get_goal();
    auto feedback = std::make_shared<CleaningTask::Feedback>();
    auto result   = std::make_shared<CleaningTask::Result>();

    {
      std::lock_guard<std::mutex> lk(pose_mtx_);
      total_distance_runtime_ = 0.0;
      has_prev_ = false;
    }

    std::vector<std::pair<double,double>> waypoints;

    turtlesim::msg::Pose start{};
    {
      std::lock_guard<std::mutex> lk(pose_mtx_);
      start = pose_;
    }

    if (goal->task_type == "clean_square") {
      const double S = goal->area_size;
      const double step = 0.5;
      const int nx = static_cast<int>(std::floor(S/step)) + 1;
      const int ny = static_cast<int>(std::floor(S/step)) + 1;
      const double x0 = start.x;
      const double y0 = start.y;

      for (int iy = 0; iy < ny; ++iy) {
        const double y = y0 + iy * step;
        if (iy % 2 == 0) {
          for (int ix = 0; ix < nx; ++ix) waypoints.emplace_back(x0 + ix*step, y);
        } else {
          for (int ix = nx - 1; ix >= 0; --ix) waypoints.emplace_back(x0 + ix*step, y);
        }
      }
    }
    else {
      waypoints.emplace_back(goal->target_x, goal->target_y);
    }

    const int total_points = static_cast<int>(waypoints.size());
    int cleaned_points = 0;

    // основной цикл
    for (int i = 0; i < total_points; ++i) {
      if (gh->is_canceling()) {
        stop_external_mtg();
        send_zero_twist();
        result->success = false;
        result->cleaned_points = cleaned_points;
        result->total_distance = current_total_distance();
        gh->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal was cancelled by user.");
        return;
      }

      double target_x = waypoints[i].first;
      double target_y = waypoints[i].second;
      double target_theta = 0.0;
      {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        if (i + 1 < total_points) {
          double nx = waypoints[i+1].first - target_x;
          double ny = waypoints[i+1].second - target_y;
          target_theta = std::atan2(ny, nx);
        } else {
          target_theta = pose_.theta;
        }
      }

      bool ok = run_move_to_goal_process(gh, target_x, target_y, target_theta,
                                         cleaned_points, total_points);
      if (!ok) {
        stop_external_mtg();
        send_zero_twist();
        result->success = false;
        result->cleaned_points = cleaned_points;
        result->total_distance = current_total_distance();
        gh->abort(result);
        RCLCPP_WARN(get_logger(), "Couldn't reach the point #%d (%.2f, %.2f) using move_to_goal.",
                    i, target_x, target_y);
        return;
      }

      cleaned_points++;

      // отправим финальный feedback по точке
      turtlesim::msg::Pose cur{};
      {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        cur = pose_;
      }
      feedback->current_cleaned_points = cleaned_points;
      feedback->progress_percent =
        static_cast<int>(std::round(100.0 * cleaned_points / std::max(1, total_points)));
      feedback->current_x = cur.x;
      feedback->current_y = cur.y;
      gh->publish_feedback(feedback);
    }

    stop_external_mtg();
    send_zero_twist();

    result->success = true;
    result->cleaned_points = cleaned_points;
    result->total_distance = current_total_distance();
    gh->succeed(result);

    RCLCPP_INFO(get_logger(), "Finished: points=%d, distance=%.3f",
                cleaned_points, result->total_distance);
  }

  bool run_move_to_goal_process(const std::shared_ptr<GoalHandle>& gh,
                                double x, double y, double theta,
                                int cleaned_points, int total_points)
  {
    std::string sx = std::to_string(x);
    std::string sy = std::to_string(y);
    std::string sth = std::to_string(theta);

    std::vector<char*> argv;
    argv.push_back(const_cast<char*>("ros2"));
    argv.push_back(const_cast<char*>("run"));
    argv.push_back(const_cast<char*>("move_to_goal"));
    argv.push_back(const_cast<char*>("move_to_goal"));
    argv.push_back(const_cast<char*>(sx.c_str()));
    argv.push_back(const_cast<char*>(sy.c_str()));
    argv.push_back(const_cast<char*>(sth.c_str()));
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(get_logger(), "Couldn't fork the process");
      return false;
    }
    if (pid == 0) {
      execvp("ros2", argv.data());
      _exit(127);
    }

    {
      std::lock_guard<std::mutex> lk(child_mtx_);
      child_pid_ = pid;
    }

    rclcpp::Rate r(20.0);

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        kill_child_if_running(SIGINT);
        return false;
      }

      int status = 0;
      pid_t rc = waitpid(pid, &status, WNOHANG);
      if (rc == pid) {
        {
          std::lock_guard<std::mutex> lk(child_mtx_);
          child_pid_ = -1;
        }
        if (WIFEXITED(status)) {
          int code = WEXITSTATUS(status);
          return (code == 0);
        } else if (WIFSIGNALED(status)) {
          int sig = WTERMSIG(status);
          RCLCPP_WARN(get_logger(), "move_to_goal is finished by signal %d", sig);
          return false;
        } else {
          return false;
        }
      }

      auto feedback = std::make_shared<CleaningTask::Feedback>();
      turtlesim::msg::Pose cur{};
      {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        cur = pose_;
      }
      feedback->current_cleaned_points = cleaned_points;
      feedback->progress_percent =
        static_cast<int>(std::round(100.0 * cleaned_points / std::max(1, total_points)));
      feedback->current_x = cur.x;
      feedback->current_y = cur.y;
      gh->publish_feedback(feedback);

      r.sleep();
    }

    kill_child_if_running(SIGINT);
    return false;
  }

  void kill_child_if_running(int sig) {
    pid_t pid_copy = -1;
    {
      std::lock_guard<std::mutex> lk(child_mtx_);
      pid_copy = child_pid_;
    }
    if (pid_copy > 0) {
      kill(pid_copy, sig);
      int status = 0;
      (void)waitpid(pid_copy, &status, 0);
      std::lock_guard<std::mutex> lk(child_mtx_);
      child_pid_ = -1;
    }
  }

  void stop_external_mtg() {
    kill_child_if_running(SIGINT);
  }

  void send_zero_twist() {
    geometry_msgs::msg::Twist zero;
    pub_cmd_->publish(zero);
  }

  double current_total_distance() const {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    return total_distance_runtime_;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_pose_;
  rclcpp_action::Server<CleaningTask>::SharedPtr action_server_;

  mutable std::mutex pose_mtx_;
  turtlesim::msg::Pose pose_{};
  turtlesim::msg::Pose prev_pose_{};
  bool have_pose_{false};
  bool has_prev_{false};
  double total_distance_runtime_{0.0};

  mutable std::mutex child_mtx_;
  pid_t child_pid_{-1};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CleaningActionServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

