#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class SpawnTurtle : public rclcpp::Node {
public:
  SpawnTurtle() : Node("spawn_turtle") {
    turtle_name_ = this->declare_parameter<std::string>("turtle_name", "turtle2");
    x_     = this->declare_parameter<double>("x", 8.0);
    y_     = this->declare_parameter<double>("y", 8.0);
    theta_ = this->declare_parameter<double>("theta", 0.0);

    client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    timer_ = this->create_wall_timer(200ms, std::bind(&SpawnTurtle::trySpawn, this));
  }

private:
  void trySpawn() {
    if (sent_) return;
    if (!client_->wait_for_service(0s)) return;

    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    req->x = x_; req->y = y_; req->theta = theta_; req->name = turtle_name_;

    sent_ = true;
    auto cb = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture fut) {
      try {
        auto resp = fut.get();
        RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", resp->name.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Spawn error: %s", e.what());
      }
      rclcpp::shutdown();
    };

    (void)client_->async_send_request(req, cb);
  }

  bool sent_{false};
  std::string turtle_name_;
  double x_{8.0}, y_{8.0}, theta_{0.0};
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpawnTurtle>());
  return 0;
}
