#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "r4c_navigation/action/path_navigation.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = rclcpp::Node::make_shared("action_server_cancellation", "", node_options);

  // Create path_navigation action client 
  rclcpp_action::Client<r4c_navigation::action::PathNavigation>::SharedPtr
    client_ptr = rclcpp_action::create_client<r4c_navigation::action::PathNavigation>(
      node,
      "path_navigation");

  // Wait for action server to be availabe
  if(!client_ptr->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  // Cancel all goals
  client_ptr->async_cancel_all_goals();
  RCLCPP_WARN(node->get_logger(), "Cancellation was requested.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}