#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

class TurtleController : public rclcpp::Node {
public:
  TurtleController() : Node("turtle_controller") {
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle2");
    switch_threshold_ = this->declare_parameter<double>("switch_threshold", 1.0);

    // Имена целей по умолчанию
    targets_ = {
      this->declare_parameter<std::string>("target0", "carrot"),
      this->declare_parameter<std::string>("target1", "carrot2"),
      this->declare_parameter<std::string>("target2", "static_target")
    };

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    vel_pub_     = this->create_publisher<geometry_msgs::msg::Twist>(turtlename_ + "/cmd_vel", 10);

    current_pub_ = this->create_publisher<std_msgs::msg::String>("/current_target", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&TurtleController::onTimer, this));

    // Отдельный поток для клавиши 'n'
    key_thread_ = std::thread([this]() { this->keyboardLoop(); });

    RCLCPP_INFO(this->get_logger(),
      "Controller for %s: threshold=%.2f; targets=[%s -> %s -> %s -> ...]",
      turtlename_.c_str(), switch_threshold_,
      targets_[0].c_str(), targets_[1].c_str(), targets_[2].c_str());

    publishCurrentTarget(); // объявим текущую цель в топике
  }

  ~TurtleController() override {
    stop_keys_.store(true);
    if (key_thread_.joinable()) key_thread_.join();
    restoreTerminal();
  }

private:
  // Основной цикл управления движением
  void onTimer() {
    // Проверка наличия TF до текущей цели
    const std::string& target = targets_[idx_];
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(turtlename_, target, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      // Нет трансформа — стоим на месте
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No TF %s -> %s: %s", turtlename_.c_str(), target.c_str(), ex.what());
      vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;

    double dist  = std::hypot(x, y);
    double angle = std::atan2(y, x);

    // Простой P-контроллер
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = std::min(2.0, 1.5 * dist);
    cmd.angular.z = 4.0 * angle;
    vel_pub_->publish(cmd);

    // Автопереключение по порогу
    if (dist < switch_threshold_) {
      nextTarget("auto (threshold)");
    }

    // Ручное переключение по клавише 'n'
    if (manual_switch_.exchange(false)) {
      nextTarget("manual ('n')");
    }
  }

  void nextTarget(const char* reason) {
    idx_ = (idx_ + 1) % targets_.size();
    RCLCPP_INFO(this->get_logger(), "Switch target to [%s] by %s",
                targets_[idx_].c_str(), reason);
    publishCurrentTarget();
  }

  void publishCurrentTarget() {
    std_msgs::msg::String msg;
    msg.data = targets_[idx_];
    current_pub_->publish(msg);
  }

  // --------- обработка клавиатуры ('n') ----------
  void keyboardLoop() {
    if (!makeTerminalRaw()) {
      RCLCPP_WARN(this->get_logger(), "Keyboard control disabled (failed to set raw terminal).");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Press 'n' to switch target.");

    while (!stop_keys_.load()) {
      char c = 0;
      ssize_t r = read(STDIN_FILENO, &c, 1);
      if (r == 1) {
        if (c == 'n' || c == 'N') {
          manual_switch_.store(true);
        }
      } else {
        // чуть поспим
        std::this_thread::sleep_for(10ms);
      }
    }
  }

  bool makeTerminalRaw() {
    if (tcgetattr(STDIN_FILENO, &orig_term_) == -1) return false;
    termios raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1; // 0.1s
    return tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }

  void restoreTerminal() {
    if (orig_term_.c_cflag != 0) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
      orig_term_ = termios{}; // обнулить
    }
  }

  // ------------- поля -------------
  std::string turtlename_;
  double switch_threshold_{1.0};

  std::vector<std::string> targets_;
  size_t idx_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<bool> manual_switch_{false};
  std::atomic<bool> stop_keys_{false};
  std::thread key_thread_;
  termios orig_term_{}; // сохранённые настройки терминала
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
}

