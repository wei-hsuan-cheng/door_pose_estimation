#include <exception>
#include <functional>
#include <set>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
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
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    declareParameterIfMissing<std::string>("action_name", "estimate_door_poses");
    declareParameterIfMissing<std::string>("door_handle_frame_id", "door_handle");
    declareParameterIfMissing<std::string>("door_hinge_frame_id", "door_hinge");
    declareParameterIfMissing<bool>("broadcast_result_tf", false);
    declareParameterIfMissing<std::vector<std::string>>(
      "generated_frame_ids", std::vector<std::string>{"door_handle", "door_hinge"});
    declareParameterIfMissing<std::vector<double>>(
      "generated_frames.door_handle.pose", std::vector<double>{0.0, -0.35, 0.0, 0.0, 0.0, 0.0});
    declareParameterIfMissing<std::vector<double>>(
      "generated_frames.door_hinge.pose", std::vector<double>{0.0, 0.45, 0.0, 0.0, 0.0, 0.0});
    action_name_ = this->get_parameter("action_name").as_string();

    action_server_ = rclcpp_action::create_server<EstimateDoorPoses>(
      this,
      action_name_,
      std::bind(&DoorPoseEstimationActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoorPoseEstimationActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&DoorPoseEstimationActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Door pose estimation action server ready on %s", action_name_.c_str());
  }

private:
  template <typename ParameterT>
  void declareParameterIfMissing(const std::string & name, const ParameterT & default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<ParameterT>(name, default_value);
    }
  }

  struct FrameConfig {
    std::string frame_id;
    tf2::Transform relative_transform;
  };

  tf2::Transform parsePose(const std::vector<double> & pose, const std::string & parameter_name) const {
    if (pose.size() != 6 && pose.size() != 7) {
      std::ostringstream error;
      error << "Parameter '" << parameter_name
            << "' must contain 6 values [x, y, z, thz, thy, thx] or 7 values "
               "[x, y, z, qw, qx, qy, qz], but got "
            << pose.size() << ".";
      throw std::runtime_error(error.str());
    }

    tf2::Quaternion rotation;
    if (pose.size() == 6) {
      rotation.setRPY(pose[5], pose[4], pose[3]);
    } else {
      rotation = tf2::Quaternion(pose[4], pose[5], pose[6], pose[3]);
    }

    if (rotation.length2() <= std::numeric_limits<double>::epsilon()) {
      throw std::runtime_error("Pose rotation must not be a zero-length quaternion.");
    }

    rotation.normalize();
    return tf2::Transform(rotation, tf2::Vector3(pose[0], pose[1], pose[2]));
  }

  std::vector<FrameConfig> readGeneratedFrames() const {
    const auto frame_ids = this->get_parameter("generated_frame_ids").as_string_array();
    if (frame_ids.empty()) {
      throw std::runtime_error("Parameter 'generated_frame_ids' must contain at least one frame.");
    }
    if (std::set<std::string>(frame_ids.begin(), frame_ids.end()).size() != frame_ids.size()) {
      throw std::runtime_error("Parameter 'generated_frame_ids' must not contain duplicates.");
    }

    std::vector<FrameConfig> frame_configs;
    frame_configs.reserve(frame_ids.size());

    for (const auto & frame_id : frame_ids) {
      if (frame_id.empty()) {
        throw std::runtime_error("Parameter 'generated_frame_ids' must not contain empty frame IDs.");
      }

      const auto parameter_name = "generated_frames." + frame_id + ".pose";
      std::vector<double> pose;
      if (!this->get_parameter(parameter_name, pose)) {
        throw std::runtime_error("Missing required parameter '" + parameter_name + "'.");
      }

      frame_configs.push_back(FrameConfig{frame_id, parsePose(pose, parameter_name)});
    }

    return frame_configs;
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
    if (goal->parent_frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: parent_frame_id is empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->child_frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: child_frame_id is empty.");
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
    geometry_msgs::msg::TransformStamped door_panel_tf;

    feedback->status = "COMPUTING";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled before pose computation.";
      goal_handle->canceled(result);
      return;
    }

    try {
      door_panel_tf.header.frame_id = goal->parent_frame_id;
      door_panel_tf.header.stamp = this->now();
      door_panel_tf.child_frame_id = goal->child_frame_id;
      door_panel_tf.transform = goal->transform;

      const auto handle_frame_id = this->get_parameter("door_handle_frame_id").as_string();
      const auto hinge_frame_id = this->get_parameter("door_hinge_frame_id").as_string();
      const bool broadcast_result_tf = this->get_parameter("broadcast_result_tf").as_bool();
      const auto generated_frames = readGeneratedFrames();
      std::vector<geometry_msgs::msg::TransformStamped> composed_transforms;
      composed_transforms.reserve(generated_frames.size());
      std::map<std::string, geometry_msgs::msg::TransformStamped> composed_transform_map;

      for (const auto & generated_frame : generated_frames) {
        auto composed_transform = composeTransform(
          door_panel_tf, generated_frame.relative_transform, generated_frame.frame_id);
        composed_transform_map[generated_frame.frame_id] = composed_transform;
        composed_transforms.push_back(std::move(composed_transform));
      }

      const auto handle_it = composed_transform_map.find(handle_frame_id);
      if (handle_it == composed_transform_map.end()) {
        throw std::runtime_error(
          "Configured door_handle_frame_id '" + handle_frame_id +
          "' is not present in generated_frame_ids.");
      }

      const auto hinge_it = composed_transform_map.find(hinge_frame_id);
      if (hinge_it == composed_transform_map.end()) {
        throw std::runtime_error(
          "Configured door_hinge_frame_id '" + hinge_frame_id +
          "' is not present in generated_frame_ids.");
      }

      result->door_handle = handle_it->second;
      result->door_hinge = hinge_it->second;
      result->success = true;
      result->message =
        "Generated " + std::to_string(composed_transforms.size()) + " transform(s) from the door panel frame.";

      if (broadcast_result_tf) {
        tf_broadcaster_->sendTransform(composed_transforms);
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
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<DoorPoseEstimationActionServer>(options));
  rclcpp::shutdown();
  return 0;
}
