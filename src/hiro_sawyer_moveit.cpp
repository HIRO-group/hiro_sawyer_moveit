#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string PLANNING_GROUP = "right_arm";
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

void poseCallback(const geometry_msgs::Pose& msg)
{
    move_group->setPoseTarget(msg);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group->move();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hiro_move_group_interface");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/hiro/sawyer/target", 1, poseCallback);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group->setPlannerId("RRTConnect");
    ROS_INFO("Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group->getEndEffectorLink().c_str());
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 0.0;
    target_pose1.orientation.y = 1.0;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.0;
    target_pose1.position.x = 0.45;
    target_pose1.position.y = 0.45;
    target_pose1.position.z = 0.3;
    move_group->setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group->move();
    ros::waitForShutdown();
    return 0;
}