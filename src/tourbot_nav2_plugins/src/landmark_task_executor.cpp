#include "tourbot_nav2_plugins/landmark_task_executor.hpp"

#include <chrono>
#include <future>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace tourbot_nav2_plugins
{

void LandmarkTaskExecutor::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  node_ = parent.lock();

  if (!node_) {
    throw std::runtime_error("Failed to lock lifecycle node in LandmarkTaskExecutor");
  }

  logger_ = node_->get_logger();

  node_->declare_parameter(plugin_name + ".enabled", enabled_);
  node_->declare_parameter(plugin_name + ".action_name", action_name_);
  node_->declare_parameter(plugin_name + ".server_timeout_sec", server_timeout_sec_);
  node_->declare_parameter(plugin_name + ".result_timeout_sec", result_timeout_sec_);

  node_->get_parameter(plugin_name + ".enabled", enabled_);
  node_->get_parameter(plugin_name + ".action_name", action_name_);
  node_->get_parameter(plugin_name + ".server_timeout_sec", server_timeout_sec_);
  node_->get_parameter(plugin_name + ".result_timeout_sec", result_timeout_sec_);

  action_client_ = rclcpp_action::create_client<DoLandmarkTask>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    action_name_);

  RCLCPP_INFO(
    logger_,
    "LandmarkTaskExecutor initialized. action_name='%s', enabled=%s",
    action_name_.c_str(),
    enabled_ ? "true" : "false");
}

bool LandmarkTaskExecutor::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /* curr_pose */,
  const int & curr_waypoint_index)
{
  if (!enabled_) {
    RCLCPP_INFO(logger_, "LandmarkTaskExecutor disabled; skipping waypoint task.");
    return true;
  }

  RCLCPP_INFO(
    logger_,
    "Reached waypoint %d. Calling landmark task action server...",
    curr_waypoint_index);

  const auto server_timeout =
    std::chrono::duration<double>(server_timeout_sec_);

  if (!action_client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      logger_,
      "Action server '%s' not available after %.2f seconds",
      action_name_.c_str(),
      server_timeout_sec_);
    return false;
  }

  DoLandmarkTask::Goal goal;
  goal.waypoint_index = curr_waypoint_index;
  goal.expected_tag_id = -1;
  goal.expected_landmark_name = "";

  auto goal_handle_future = action_client_->async_send_goal(goal);

  if (goal_handle_future.wait_for(server_timeout) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timed out while sending landmark task goal.");
    return false;
  }

  auto goal_handle = goal_handle_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Landmark task goal was rejected.");
    return false;
  }

  auto result_future = action_client_->async_get_result(goal_handle);

  const auto result_timeout =
    std::chrono::duration<double>(result_timeout_sec_);

  if (result_future.wait_for(result_timeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      logger_,
      "Timed out waiting for landmark task result at waypoint %d.",
      curr_waypoint_index);
    return false;
  }

  auto wrapped_result = result_future.get();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      logger_,
      "Landmark task action did not succeed. Result code: %d",
      static_cast<int>(wrapped_result.code));
    return false;
  }

  auto result = wrapped_result.result;

  if (!result->success) {
    RCLCPP_ERROR(
      logger_,
      "Landmark task failed at waypoint %d. Detected tag: %d. Message: %s",
      curr_waypoint_index,
      result->detected_tag_id,
      result->message.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "Landmark task succeeded at waypoint %d. Detected tag: %d. Message: %s",
    curr_waypoint_index,
    result->detected_tag_id,
    result->message.c_str());

  return true;
}

}  // namespace tourbot_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  tourbot_nav2_plugins::LandmarkTaskExecutor,
  nav2_core::WaypointTaskExecutor)