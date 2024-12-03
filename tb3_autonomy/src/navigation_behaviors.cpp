#include "navigation_behaviors.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <chrono>

using namespace std::chrono_literals;

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
        node_ptr_, 
        "/carter1/navigate_to_pose"  // Updated to match your working action
    );
    done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
    // Wait for action server
    if (!action_client_ptr_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Navigation action server not available after waiting");
        return BT::NodeStatus::FAILURE;
    }

    // Get location from YAML
    BT::Optional<std::string> loc = getInput<std::string>("loc");
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

    RCLCPP_INFO(node_ptr_->get_logger(), 
                "Navigating to %s: [x: %.2f, y: %.2f, theta: %.2f]",
                loc.value().c_str(), pose[0], pose[1], pose[2]);

    // Create the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];
    goal_msg.pose.pose.position.z = 0.0;

    // Convert theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Setup callbacks
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    // Send the goal
    done_flag_ = false;
    auto future_goal_handle = action_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "Goal sent to navigation server");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
    if (done_flag_) {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Navigation goal reached", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Navigation goal succeeded");
        done_flag_ = true;
    } else {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Navigation goal failed with status code: %d", 
                     static_cast<int>(result.code));
        done_flag_ = false;
    }
}