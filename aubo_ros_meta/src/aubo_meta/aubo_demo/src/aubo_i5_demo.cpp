#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

namespace rvt = rviz_visual_tools;

/**
 * @brief demo: add obstacle and plan path to avoid obstacle
 * 
 * @param visual_tools 
 * @param move_group 
 * @param planning_scene_interface 
 * @param joint_model_group 
 * @param text_pose 
 */
void demo06_collision_objects(
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const robot_state::JointModelGroup *joint_model_group,
    Eigen::Isometry3d &text_pose)
{
    visual_tools.prompt("Step 13: Press 'next' to start demo: collision objects");

    // define collision_object1
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group.getPlanningFrame();
    collision_object1.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.05;
    primitive.dimensions[2] = 0.4;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.3;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.54;
    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(box_pose);
    collision_object1.operation = collision_object1.ADD;

    // define collision_object2
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group.getPlanningFrame();
    collision_object2.id = "box2";
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 1.7;
    primitive2.dimensions[1] = 1.7;
    primitive2.dimensions[2] = 0.05;
    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0.0;
    box_pose2.position.y = 0.0;
    box_pose2.position.z = 0.0;
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;

    // add collision objects to the scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object1);
    collision_objects.push_back(collision_object2);
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "AUBO Add object Example5", rvt::RED, rvt::XLARGE);
    visual_tools.trigger();

    // set start state
    visual_tools.prompt("Step 14: Press 'next' to go to obstacle path start pose");
    move_group.setStartState(*move_group.getCurrentState());

    // set target pose
    tf::Quaternion target_pose_q;
    target_pose_q.setRPY(1.77, -0.59, -1.79); // radian
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = target_pose_q.x();
    target_pose.orientation.y = target_pose_q.y();
    target_pose.orientation.z = target_pose_q.z();
    target_pose.orientation.w = target_pose_q.w();
    target_pose.position.x = -0.37;
    target_pose.position.y = 0.6;
    target_pose.position.z = 0.4;
    move_group.setPoseTarget(target_pose);

    // path plan
    visual_tools.prompt("Step 15: Press 'next' to start path_collision_avoid_plan planning");
    moveit::planning_interface::MoveGroupInterface::Plan path_collision_avoid_plan;
    bool success = (move_group.plan(path_collision_avoid_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing path_collision_avoid_plan %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "AUBO Obstacle Goal Exalmple6", rvt::RED, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(path_collision_avoid_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Perform planning actions
    visual_tools.prompt("Step 16: Press 'next' to execute path_collision_avoid_plan");
    move_group.execute(path_collision_avoid_plan);

    // Move to the home point position
    visual_tools.prompt("Step 17: Press 'next' to go home");
    std::vector<double> home_pose;
    home_pose.push_back(-0.001255);
    home_pose.push_back(-0.148822);
    home_pose.push_back(-1.406503);
    home_pose.push_back(0.311441);
    home_pose.push_back(-1.571295);
    home_pose.push_back(-0.002450);
    move_group.setJointValueTarget(home_pose);
    move_group.move();
}

/**
 * @brief Demo: cartesian path planning with interpolation
 *
 * @param visual_tools
 * @param move_group
 * @param joint_model_group
 * @param text_pose
 */
void demo05_cartesian_interpolation(
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const robot_state::JointModelGroup *joint_model_group,
    Eigen::Isometry3d &text_pose)
{
    visual_tools.prompt("Step 11: Press 'next' to start demo: cartesian interpolation");

    //  waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose way_pose_1;
    way_pose_1.position.x = -0.4;
    way_pose_1.position.y = 0.05;
    way_pose_1.position.z = 0.54;
    tf::Quaternion way_pose_1_q;
    way_pose_1_q.setRPY(3.14, 0, -1.57);
    way_pose_1.orientation.x = way_pose_1_q.x();
    way_pose_1.orientation.y = way_pose_1_q.y();
    way_pose_1.orientation.z = way_pose_1_q.z();
    way_pose_1.orientation.w = way_pose_1_q.w();
    waypoints.push_back(way_pose_1);
    // down
    geometry_msgs::Pose way_pose_2 = way_pose_1;
    way_pose_2.position.z -= 0.2;
    waypoints.push_back(way_pose_2);
    // right
    way_pose_2.position.y -= 0.15;
    waypoints.push_back(way_pose_2);
    // up and left
    way_pose_2.position.z += 0.2;
    way_pose_2.position.y += 0.2;
    way_pose_2.position.x -= 0.2;
    waypoints.push_back(way_pose_2);

    // reduce speed
    move_group.setMaxVelocityScalingFactor(0.5);

    // set interpolation param
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; //(The jump threshold is set to 0.0)
    const double eef_step = 0.01;      //(interpolation step)

    // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }
    visual_tools.trigger();

    // execute path_interpolation_plan
    visual_tools.prompt("Step 12: Press 'next' to execute path_interpolation_plan");
    moveit::planning_interface::MoveGroupInterface::Plan path_interpolation_plan;
    path_interpolation_plan.trajectory_ = trajectory;
    move_group.execute(path_interpolation_plan);

    // go home
    visual_tools.prompt("Step 13: Press 'next' to go home");
    std::vector<double> home_pose;
    home_pose.push_back(-0.001255);
    home_pose.push_back(-0.148822);
    home_pose.push_back(-1.406503);
    home_pose.push_back(0.311441);
    home_pose.push_back(-1.571295);
    home_pose.push_back(-0.002450);
    move_group.setJointValueTarget(home_pose);
    move_group.move();
}

/**
 * @brief Demo: cartesian path planning with path constraints
 *
 * @param visual_tools
 * @param move_group
 * @param joint_model_group
 * @param text_pose
 */
void demo04_cartesian_path_planning_with_constraints(
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const robot_state::JointModelGroup *joint_model_group,
    Eigen::Isometry3d &text_pose)
{
    visual_tools.prompt("Step 7: Press 'next' to start cartesian path planning with path constraints");

    // set path constraint
    moveit_msgs::OrientationConstraint oc;
    oc.link_name = "gripper_tip_Link";
    oc.header.frame_id = "base_link";
    tf::Quaternion oc_q;
    oc_q.setRPY(3.14, 0, -1.57);
    oc.orientation.w = oc_q.w();
    oc.orientation.x = oc_q.x();
    oc.orientation.y = oc_q.y();
    oc.orientation.z = oc_q.z();
    oc.absolute_x_axis_tolerance = 0.2;
    oc.absolute_y_axis_tolerance = 0.2;
    oc.absolute_z_axis_tolerance = 0.2;
    oc.weight = 1.0;

    // add path_constraints
    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(oc);
    move_group.setPathConstraints(path_constraints);

    // set path start pose
    geometry_msgs::Pose path_start_pose;
    path_start_pose.position.x = -0.4;
    path_start_pose.position.y = 0.05;
    path_start_pose.position.z = 0.3;
    path_start_pose.orientation.x = oc_q.x();
    path_start_pose.orientation.y = oc_q.y();
    path_start_pose.orientation.z = oc_q.z();
    path_start_pose.orientation.w = oc_q.w();

    // move to path start pose
    visual_tools.prompt("Step 8: Press 'next' to move to path start pose");
    move_group.setPoseTarget(path_start_pose);
    move_group.move();

    // set start state
    // robot_state::RobotState start_state(*move_group.getCurrentState());
    // start_state.setFromIK(joint_model_group, path_start_pose); // get IK ?
    move_group.setStartStateToCurrentState();

    // set path_end_pose
    visual_tools.prompt("Step 9: Press 'next' to plan path with constraints");
    geometry_msgs::Pose path_end_pose;
    path_end_pose.position.x = 0.4;
    path_end_pose.position.y = 0.4;
    path_end_pose.position.z = 0.4;
    path_end_pose.orientation.x = oc_q.x();
    path_end_pose.orientation.y = oc_q.y();
    path_end_pose.orientation.z = oc_q.z();
    path_end_pose.orientation.w = oc_q.w();
    move_group.setPoseTarget(path_end_pose);

    // move plan with larger planning time
    move_group.setPlanningTime(20.0);
    moveit::planning_interface::MoveGroupInterface::Plan path_constraints_plan;
    bool success = (move_group.plan(path_constraints_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing path_constraints_plan (constraints) %s", success ? "success" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(path_start_pose, "path_start_pose");
    visual_tools.publishAxisLabeled(path_end_pose, "path_end_pose");
    visual_tools.publishText(text_pose, "AUBO Constrained Goal Example3", rvt::RED, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(path_constraints_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Perform planning actions
    visual_tools.prompt("Step 10: Press 'next' to execute path_constraints_plan");
    move_group.execute(path_constraints_plan);

    // clear path constraints
    move_group.clearPathConstraints();
}

/**
 * @brief joint move with specified joint angle
 *
 * @param visual_tools
 * @param move_group
 * @param joint_model_group
 * @param text_pose
 */
void demo03_joint_angle_move(
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    moveit::planning_interface::MoveGroupInterface &move_group,
    const robot_state::JointModelGroup *joint_model_group,
    Eigen::Isometry3d &text_pose)
{
    visual_tools.prompt("Step 5: Press 'next' to execute joint space move plan demo");

    // get joint values using copyJointGroupPositions()
    std::vector<double> joint_group_positions;
    // get current robot state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    ROS_INFO_STREAM("joint_group_positions");
    for (size_t i = 0; i < joint_group_positions.size(); i++)
    {
        ROS_INFO_STREAM(joint_group_positions[i]);
    }
    // same as current_joint_values
    ROS_INFO_STREAM("current_joint_values");
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    for (size_t i = 0; i < current_joint_values.size(); i++)
    {
        ROS_INFO_STREAM(current_joint_values[i]);
    }

    // modify joint 1 angle
    current_joint_values[0] += 90.0 / 180.0 * M_PI;

    // set joint value target
    move_group.setJointValueTarget(current_joint_values);

    // move plan
    moveit::planning_interface::MoveGroupInterface::Plan target_pose_2_plan;
    bool success = (move_group.plan(target_pose_2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing target_pose_2_plan (joint space goal) %s", success ? "success" : "FAILED");

    // Visual display in RVIZ
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example2", rvt::RED, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(target_pose_2_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Perform planning actions
    visual_tools.prompt("Step 6: Press 'next' to execute target_pose_2_plan");
    move_group.execute(target_pose_2_plan);

    // go home
    visual_tools.prompt("Step 6: Press 'next' to go to home position");
    std::vector<double> home_pose;
    home_pose.push_back(-0.001255);
    home_pose.push_back(-0.148822);
    home_pose.push_back(-1.406503);
    home_pose.push_back(0.311441);
    home_pose.push_back(-1.571295);
    home_pose.push_back(-0.002450);
    move_group.setJointValueTarget(home_pose);
    move_group.move();
}

/**
 * @brief Demo: get robot state
 *
 * @param visual_tools
 * @param move_group
 */
void demo02_get_robot_state(
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    moveit::planning_interface::MoveGroupInterface &move_group)
{
    visual_tools.prompt("Step 4: Press 'next' to get current robot state");

    // get current robot state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // get active joints
    ros::V_string active_joints = move_group.getActiveJoints();
    ROS_INFO_STREAM("active_joints: ");
    for (size_t i = 0; i < active_joints.size(); i++)
    {
        ROS_INFO_STREAM(active_joints[i]);
    }

    // get current joint values
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    ROS_INFO_STREAM("current_joint_values: ");
    for (size_t i = 0; i < current_joint_values.size(); i++)
    {
        ROS_INFO_STREAM(current_joint_values[i]);
    }

    // get current pose
    // Note: this pose is based on world link, not base_link
    geometry_msgs::PoseStamped current_pose_world = move_group.getCurrentPose();
    ROS_INFO_STREAM("current_pose_world: " << current_pose_world);

    // get TF transform base_link -> gripper_tip_Link
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("base_link", "gripper_tip_Link", ros::Time(0), ros::Duration(2.0));
    tf::StampedTransform current_pose_base_link_tf;
    geometry_msgs::PoseStamped current_pose_base_link;
    tf_listener.lookupTransform("base_link", "gripper_tip_Link", ros::Time(0), current_pose_base_link_tf);
    current_pose_base_link.pose.position.x = current_pose_base_link_tf.getOrigin().getX();
    current_pose_base_link.pose.position.y = current_pose_base_link_tf.getOrigin().getY();
    current_pose_base_link.pose.position.z = current_pose_base_link_tf.getOrigin().getZ();
    current_pose_base_link.pose.orientation.w = current_pose_base_link_tf.getRotation().getW();
    current_pose_base_link.pose.orientation.x = current_pose_base_link_tf.getRotation().getX();
    current_pose_base_link.pose.orientation.y = current_pose_base_link_tf.getRotation().getY();
    current_pose_base_link.pose.orientation.z = current_pose_base_link_tf.getRotation().getZ();
    ROS_INFO_STREAM("current_pose_base_link: " << current_pose_base_link);

    // get current rpy
    std::vector<double> current_rpy = move_group.getCurrentRPY();
    ROS_INFO_STREAM("current_rpy: ");
    for (size_t i = 0; i < current_rpy.size(); i++)
    {
        ROS_INFO_STREAM(current_rpy[i]);
    }
}

/**
 * @brief Demo: joint move, cartesian move
 * home_pose -> target_pose_1 -> home_pose
 *
 * @param visual_tools
 * @param move_group
 * @param text_pose
 * @param joint_model_group
 */
void demo01_joint_cartesian_move(
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    moveit::planning_interface::MoveGroupInterface &move_group,
    Eigen::Isometry3d &text_pose,
    const robot_state::JointModelGroup *joint_model_group)
{
    // go to home_pose
    visual_tools.prompt("Step 1: Press 'next' to go to home position");
    std::vector<double> home_pose;
    home_pose.push_back(-0.001255);
    home_pose.push_back(-0.148822);
    home_pose.push_back(-1.406503);
    home_pose.push_back(0.311441);
    home_pose.push_back(-1.571295);
    home_pose.push_back(-0.002450);
    move_group.setJointValueTarget(home_pose);
    move_group.move();

    // Set target_pose_1
    visual_tools.prompt("Step 2: Press 'next' to excute target_pose_1_plan");
    tf::Quaternion target_pose_1_q;
    target_pose_1_q.setRPY(3.14, 0, -1.57); // radian
    geometry_msgs::Pose target_pose_1;
    target_pose_1.position.x = -0.4;
    target_pose_1.position.y = -0.3;
    target_pose_1.position.z = 0.50;
    target_pose_1.orientation.x = target_pose_1_q.x();
    target_pose_1.orientation.y = target_pose_1_q.y();
    target_pose_1.orientation.z = target_pose_1_q.z();
    target_pose_1.orientation.w = target_pose_1_q.w();
    move_group.setPoseTarget(target_pose_1);
    ROS_INFO_STREAM("target_pose_1: " << target_pose_1);

    // calc target_pose_1_plan
    moveit::planning_interface::MoveGroupInterface::Plan target_pose_1_plan;
    bool success = (move_group.plan(target_pose_1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing target_pose_1_plan (cartesian space goal) %s", success ? "Success" : "FAILED");

    // visual planning path in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose_1, "pose1");
    visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
    // Parameter 1 (trajectory_): path information
    // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
    visual_tools.publishTrajectoryLine(target_pose_1_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // execute target_pose_1_plan
    move_group.execute(target_pose_1_plan);

    // go to home_pose
    visual_tools.prompt("Step 3: Press 'next' to go to home position");
    move_group.setJointValueTarget(home_pose);
    move_group.move();
}

/**
 * @brief aubo_i5_demo
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    // init ros
    ros::init(argc, argv, "aubo_i5_demo");
    ros::NodeHandle node_handle;

    // Start a thread, could use more thread for multi subscriber
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // init MoveGroupInterface
    static const std::string PLANNING_GROUP = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("base_link");

    // init PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // JointModelGroup
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // init MoveItVisualTools
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // create text
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.2;
    visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
    // Text visualization takes effect
    visual_tools.trigger();

    // Get the coordinate system of the basic information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // Get the end of the basic information
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    /**
     * @brief Demo: joint move, cartesian move
     *
     * home_pose -> target_pose_1 -> home_pose
     *
     */
    demo01_joint_cartesian_move(visual_tools, move_group, text_pose, joint_model_group);

    /**
     * @brief Demo: get robot state
     *
     */
    demo02_get_robot_state(visual_tools, move_group);

    /**
     * @brief Demo: joint move with specified joint angle
     *
     */
    demo03_joint_angle_move(visual_tools, move_group, joint_model_group, text_pose);

    /**
     * @brief Demo: cartesian path planning with path constraints
     *
     */
    demo04_cartesian_path_planning_with_constraints(visual_tools, move_group, joint_model_group, text_pose);

    /**
     * @brief Demo: cartesian path plannning with interpolation
     *
     */
    demo05_cartesian_interpolation(visual_tools, move_group, joint_model_group, text_pose);

    /**
     * @brief Demo: add obstacle and plan path to avoid obstacle
     * 
     */
    demo06_collision_objects(visual_tools, move_group, planning_scene_interface, joint_model_group, text_pose);

    return 0;
}
