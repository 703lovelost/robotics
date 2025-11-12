#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/summ_full_name.hpp"

#include <memory>

using SummFullName = service_full_name::srv::SummFullName;

void add(const std::shared_ptr<SummFullName::Request>  request,
               std::shared_ptr<SummFullName::Response> response)
{
  response->full_name = request->first_name + " " + request->patronym + " " + request->last_name;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: \nfirst_name: %s, patronym: %s, last_name: %s",
                request->first_name.c_str(), request->patronym.c_str(), request->last_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: %s", response->full_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("full_name_server");

  rclcpp::Service<SummFullName>::SharedPtr service = node->create_service<SummFullName>("summ_full_name", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to concat.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
