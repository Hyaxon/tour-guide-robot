#ifndef TOURBOT_NAV2_PLUGINS__LANDMARK_TASK_EXECUTOR_HPP_
#define TOURBOT_NAV2_PLUGINS__LANDMARK_TASK_EXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

#include "tourbot_interfaces/action/do_landmark_task.hpp"

namespace tourbot_nav2_plugins
{

class LandmarkTaskExecutor : public nav2_core::WaypointTaskExecutor
{
public:
  using DoLandmarkTask = tourbot_interfaces::action::DoLandmarkTask;

  LandmarkTaskExecutor() = default;
  ~LandmarkTaskExecutor() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_waypoint_index) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("LandmarkTaskExecutor")};

  rclcpp_action::Client<DoLandmarkTask>::SharedPtr action_client_;

  std::string action_name_{"/tourbot/do_landmark_task"};
  double server_timeout_sec_{5.0};
  double result_timeout_sec_{120.0};
  bool enabled_{true};
};

}  // namespace tourbot_nav2_plugins

#endif