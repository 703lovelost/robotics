#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "cleaning_robot/action/cleaning_task.hpp"

using namespace std::chrono_literals;
using cleaning_robot::action::CleaningTask;

class CleaningActionClient : public rclcpp::Node {
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<CleaningTask>;
  using WrappedResult = GoalHandle::WrappedResult;

  CleaningActionClient() : Node("cleaning_action_client") {
    client_ = rclcpp_action::create_client<CleaningTask>(this, "cleaning_task");
  }

  bool run_sequence(double side, double home_x, double home_y) {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Server is unavailable.");
      return false;
    }

    auto fb = [this](GoalHandle::SharedPtr,
                     const std::shared_ptr<const CleaningTask::Feedback> fb) {
      RCLCPP_INFO(this->get_logger(),
                  "Progress: %d%% | points=%d | pos=(%.2f, %.2f)",
                  fb->progress_percent, fb->current_cleaned_points, fb->current_x, fb->current_y);
    };

    // clean_square
    CleaningTask::Goal goal1;
    goal1.task_type = "clean_square";
    goal1.area_size = side;
    goal1.target_x = 0.0;
    goal1.target_y = 0.0;

    WrappedResult res1;
    if (!run_goal(goal1, fb, res1)) {
      RCLCPP_ERROR(get_logger(), "clean_square finished with error.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "clean_square: points=%d, dist=%.3f",
                res1.result->cleaned_points, res1.result->total_distance);

    // return_home
    CleaningTask::Goal goal2;
    goal2.task_type = "return_home";
    goal2.area_size = 0.0;
    goal2.target_x = home_x;
    goal2.target_y = home_y;

    WrappedResult res2;
    if (!run_goal(goal2, fb, res2)) {
      RCLCPP_ERROR(get_logger(), "return_home завершился неуспешно.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "return_home: dist=%.3f", res2.result->total_distance);

    return true;
  }

private:
  template<typename FeedbackCb>
  bool run_goal(const CleaningTask::Goal & goal, FeedbackCb feedback_cb, WrappedResult & out_result) {
    typename rclcpp_action::Client<CleaningTask>::SendGoalOptions opts;
    opts.feedback_callback = feedback_cb;

    auto gh_future = client_->async_send_goal(goal, opts);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(shared_from_this());

    auto rc = exec.spin_until_future_complete(gh_future, 10s);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Goal sending timeout.");
      return false;
    }

    auto gh = gh_future.get();
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "Goal refused by server.");
      return false;
    }

    auto result_future = client_->async_get_result(gh);
    rc = exec.spin_until_future_complete(result_future);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Couldn't get goal result (rc=%d).", static_cast<int>(rc));
      return false;
    }

    out_result = result_future.get();
    return (out_result.code == rclcpp_action::ResultCode::SUCCEEDED) && out_result.result->success;
  }

  rclcpp_action::Client<CleaningTask>::SharedPtr client_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  double side = 3.0, home_x = 5.5, home_y = 5.5;
  if (argc >= 2) side = std::stod(argv[1]);
  if (argc >= 4) {
    home_x = std::stod(argv[2]);
    home_y = std::stod(argv[3]);
  }

  auto node = std::make_shared<CleaningActionClient>();
  bool ok = node->run_sequence(side, home_x, home_y);
  rclcpp::shutdown();

  return ok ? 0 : 1;
}

