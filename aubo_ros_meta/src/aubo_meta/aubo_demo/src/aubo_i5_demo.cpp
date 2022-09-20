#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

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

    // Start a thread
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
    namespace rvt = rviz_visual_tools;
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
     * @brief home_pose -> target_pose_1 -> home_pose
     * 
     */

    // go to home_pose
    std::vector<double> home_pose;
    home_pose.push_back(-0.001255);
    home_pose.push_back(-0.148822);
    home_pose.push_back(-1.406503);
    home_pose.push_back(0.311441);
    home_pose.push_back(-1.571295);
    home_pose.push_back(-0.002450);
    move_group.setJointValueTarget(home_pose);
    visual_tools.prompt("1: Press 'next' to go to home position");
    move_group.move();

    // Set target_pose_1
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
    std::cout << target_pose_1;



    // calc target_pose_1_plan
    moveit::planning_interface::MoveGroupInterface::Plan target_pose_1_plan;
    bool success = (move_group.plan(target_pose_1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing target_pose_1_plan %s", success ? "Success" : "FAILED");

    // visual planning path in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose_1, "pose1");
    visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
    // Parameter 1 (trajectory_): path information
    // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
    visual_tools.publishTrajectoryLine(target_pose_1_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // execute target_pose_1_plan
    visual_tools.prompt("2: Press 'next' to excute target_pose_1_plan");
    move_group.execute(target_pose_1_plan);

    // go to home_pose
    visual_tools.prompt("3: Press 'next' to go to home position");
    move_group.setJointValueTarget(home_pose);
    move_group.move();

    return 0;
}
