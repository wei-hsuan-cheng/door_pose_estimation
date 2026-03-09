#include <array>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "door_pose_estimation/action/estimate_door_poses.hpp"

class DoorPoseEstimationActionServer : public rclcpp::Node {
public:
  using EstimateDoorPoses = door_pose_estimation::action::EstimateDoorPoses;
  using GoalHandleEstimateDoorPoses = rclcpp_action::ServerGoalHandle<EstimateDoorPoses>;

  explicit DoorPoseEstimationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("door_pose_estimation_action_server", options),
    action_name_(this->declare_parameter<std::string>("action_name", "estimate_door_poses")),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    this->declare_parameter<std::string>("door_handle_frame_id", "door_handle");
    this->declare_parameter<std::string>("door_hinge_frame_id", "door_hinge");
    this->declare_parameter<bool>("broadcast_result_tf", false);

    declareOffsetParameters("door_handle_offset", {0.0, -0.35, 0.0}, {0.0, 0.0, 0.0});
    declareOffsetParameters("door_hinge_offset", {0.0, 0.45, 0.0}, {0.0, 0.0, 0.0});

    action_server_ = rclcpp_action::create_server<EstimateDoorPoses>(
      this,
      action_name_,
      std::bind(&DoorPoseEstimationActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoorPoseEstimationActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&DoorPoseEstimationActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Door pose estimation action server ready on %s", action_name_.c_str());
  }

private:
  void declareOffsetParameters(
    const std::string & prefix,
    const std::array<double, 3> & translation_defaults,
    const std::array<double, 3> & rpy_defaults) {
    this->declare_parameter<double>(prefix + ".translation.x", translation_defaults[0]);
    this->declare_parameter<double>(prefix + ".translation.y", translation_defaults[1]);
    this->declare_parameter<double>(prefix + ".translation.z", translation_defaults[2]);
    this->declare_parameter<double>(prefix + ".rotation.roll", rpy_defaults[0]);
    this->declare_parameter<double>(prefix + ".rotation.pitch", rpy_defaults[1]);
    this->declare_parameter<double>(prefix + ".rotation.yaw", rpy_defaults[2]);
  }

  tf2::Transform readRelativeTransform(const std::string & prefix) const {
    const double x = this->get_parameter(prefix + ".translation.x").as_double();
    const double y = this->get_parameter(prefix + ".translation.y").as_double();
    const double z = this->get_parameter(prefix + ".translation.z").as_double();
    const double roll = this->get_parameter(prefix + ".rotation.roll").as_double();
    const double pitch = this->get_parameter(prefix + ".rotation.pitch").as_double();
    const double yaw = this->get_parameter(prefix + ".rotation.yaw").as_double();

    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    rotation.normalize();

    return tf2::Transform(rotation, tf2::Vector3(x, y, z));
  }

  geometry_msgs::msg::TransformStamped composeTransform(
    const geometry_msgs::msg::TransformStamped & base_transform,
    const tf2::Transform & relative_transform,
    const std::string & child_frame_id) const {
    tf2::Transform base_tf;
    tf2::fromMsg(base_transform.transform, base_tf);

    geometry_msgs::msg::TransformStamped output = base_transform;
    output.child_frame_id = child_frame_id;
    output.transform = tf2::toMsg(base_tf * relative_transform);
    return output;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const EstimateDoorPoses::Goal> goal) {
    const auto & door_panel_tf = goal->door_panel_tf;

    if (door_panel_tf.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: door_panel_tf.header.frame_id is empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (door_panel_tf.child_frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: door_panel_tf.child_frame_id is empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleEstimateDoorPoses>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleEstimateDoorPoses> goal_handle) {
    std::thread(&DoorPoseEstimationActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleEstimateDoorPoses> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<EstimateDoorPoses::Result>();
    auto feedback = std::make_shared<EstimateDoorPoses::Feedback>();

    feedback->status = "COMPUTING";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled before pose computation.";
      goal_handle->canceled(result);
      return;
    }

    try {
      const auto handle_frame_id = this->get_parameter("door_handle_frame_id").as_string();
      const auto hinge_frame_id = this->get_parameter("door_hinge_frame_id").as_string();
      const bool broadcast_result_tf = this->get_parameter("broadcast_result_tf").as_bool();

      result->door_handle_tf =
        composeTransform(goal->door_panel_tf, readRelativeTransform("door_handle_offset"), handle_frame_id);
      result->door_hinge_tf =
        composeTransform(goal->door_panel_tf, readRelativeTransform("door_hinge_offset"), hinge_frame_id);
      result->success = true;
      result->message = "Door handle and hinge transforms computed successfully.";

      if (broadcast_result_tf) {
        tf_broadcaster_->sendTransform(
          std::vector<geometry_msgs::msg::TransformStamped>{result->door_handle_tf, result->door_hinge_tf});
      }

      feedback->status = "SUCCEEDED";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
    } catch (const std::exception & ex) {
      result->success = false;
      result->message = ex.what();
      goal_handle->abort(result);
    }
  }

  std::string action_name_;
  rclcpp_action::Server<EstimateDoorPoses>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoorPoseEstimationActionServer>());
  rclcpp::shutdown();
  return 0;
}
