#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class GetPoseClient : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit GetPoseClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("get_pose_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Find>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_objects");
    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&GetPoseClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GetPoseClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GetPoseClient::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
  float X_arg;
  float Y_arg;

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFind::SharedPtr,
                         const std::shared_ptr<const Find::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  }

  void result_callback(const GoalHandleFind::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    X_arg = result.result->objects[0].object.primitive_poses[0].position.x;
    Y_arg = result.result->objects[0].object.primitive_poses[0].position.y;
    RCLCPP_INFO(this->get_logger(), "Result received");
    RCLCPP_INFO(this->get_logger(), "X: %f",
                result.result->objects[0].object.primitive_poses[0].position.x);
    RCLCPP_INFO(this->get_logger(), "Y: %f",
                result.result->objects[0].object.primitive_poses[0].position.y);
    //}
  }
}; // class GetPoseClient

static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class Trajectory : public rclcpp::Node{
    public:
        explicit Trajectory(std::shared_ptr<rclcpp::Node> move_group_node, float x_arg = 0.0, float y_arg = 0.0)
        :Node("Trajectory"),
        move_group_arm(move_group_node,PLANNING_GROUP_ARM),
        move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER),
        x_pos(x_arg),
        y_pos(y_arg) {
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "X: %f", x_pos);
        run();
      }

      void run(){
        const moveit::core::JointModelGroup *joint_model_group_arm =
            move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

        const moveit::core::JointModelGroup *joint_model_group_gripper =
            move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

        // Get Current State
        moveit::core::RobotStatePtr current_state_arm =
            move_group_arm.getCurrentState(10);
        moveit::core::RobotStatePtr current_state_gripper =
            move_group_gripper.getCurrentState(10);

        std::vector<double> joint_group_positions_arm;
        std::vector<double> joint_group_positions_gripper;
        current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                    joint_group_positions_arm);
        current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                        joint_group_positions_gripper);

        move_group_arm.setStartStateToCurrentState();
        move_group_gripper.setStartStateToCurrentState();
      }
    private:
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
        moveit::planning_interface::MoveGroupInterface move_group_arm;
        moveit::planning_interface::MoveGroupInterface move_group_gripper;
        float x_pos;
        float y_pos;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetPoseClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "X = %f", action_client->X_arg);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Y = %f", action_client->Y_arg);

  float X_pos = action_client->X_arg;
  float Y_pos = action_client->Y_arg;

//   float X_pos = 0.0;
//   float Y_pos = 0.0;
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node =
      rclcpp::Node::make_shared("move_group_perception", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto planner_node = std::make_shared<Trajectory>(move_group_node, X_pos, Y_pos);
  executor.add_node(planner_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::shutdown();
  return 0;
}

