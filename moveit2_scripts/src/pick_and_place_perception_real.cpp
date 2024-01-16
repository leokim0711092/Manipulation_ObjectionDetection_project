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


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetPoseClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "X = %f", action_client->X_arg);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Y = %f", action_client->Y_arg);

  float X_pos = action_client->X_arg;
  float Y_pos = action_client->Y_arg ;

  X_pos -= X_pos - 0.34;
  Y_pos -= Y_pos - 0.132;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "X new = %f", X_pos);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Y new = %f", Y_pos);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

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

 // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.64;
    primitive.dimensions[primitive.BOX_Y] = 0.15;
    primitive.dimensions[primitive.BOX_Z] = 2.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =1.0; 
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.55;
    box_pose.position.z = 1.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Table
  auto const collision_object_table = [frame_id =
                                           move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object_1;
    collision_object_1.header.frame_id = frame_id;
    collision_object_1.id = "box2";
    shape_msgs::msg::SolidPrimitive primitive_1;

    // Define the size of the box in meters
    primitive_1.type = primitive_1.BOX;
    primitive_1.dimensions.resize(3);
    primitive_1.dimensions[primitive_1.BOX_X] = 1.8;
    primitive_1.dimensions[primitive_1.BOX_Y] = 0.6;
    primitive_1.dimensions[primitive_1.BOX_Z] = 0.02;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose_1;
    box_pose_1.orientation.w =1.0; 
    box_pose_1.position.x = 0.8;
    box_pose_1.position.y = 0.2;
    box_pose_1.position.z = -0.011;

    collision_object_1.primitives.push_back(primitive_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;

    return collision_object_1;
  }();

  // Create collision object -- cube for the robot to avoid
  auto const collision_object_cube = [frame_id =
                                          move_group_arm.getPlanningFrame(), X_pos, Y_pos] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "cube";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.02;
    primitive.dimensions[primitive.BOX_Y] = 0.02;
    primitive.dimensions[primitive.BOX_Z] = 0.06;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    box_pose.position.x = X_pos;
    box_pose.position.y = Y_pos;
    box_pose.position.z = 0.03;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_2;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_3;

  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface_2.applyCollisionObject(collision_object_table);
  planning_scene_interface_3.applyCollisionObject(collision_object_cube);

  RCLCPP_INFO(LOGGER, "Create Collision");

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("close");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  move_group_arm.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  //   Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;

  target_pose1.position.x = X_pos;
  target_pose1.position.y = Y_pos;
  target_pose1.position.z = 0.30;

  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  /* First, define the REMOVE object message*/
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "cube";
  remove_object.header.frame_id = "world";
  remove_object.operation = remove_object.REMOVE;
  std::vector<std::string> object_ids;
  object_ids.push_back("cube");

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.14;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.14;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  //   Remove  Collision Object
  planning_scene_interface_3.removeCollisionObjects(object_ids);

  //   Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  //    Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.22;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.22;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  // Rotate

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  if (joint_group_positions_arm[0] < 0)
    joint_group_positions_arm[0] +=
        3.14; 
  else joint_group_positions_arm[0] -= 3.14;

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;
}
