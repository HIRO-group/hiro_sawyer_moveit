#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <intera_core_msgs/IODeviceStatus.h>
#include <intera_core_msgs/IONodeStatus.h>
#include <intera_core_msgs/IONodeConfiguration.h>
#include <intera_core_msgs/IODeviceConfiguration.h>
#include <intera_core_msgs/IOComponentCommand.h>
#include <intera_core_msgs/IOStatus.h>

using namespace intera_core_msgs;

std::shared_ptr<ros::NodeHandle> n;

static const std::string PLANNING_GROUP = "right_arm";
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

ros::Publisher pub_end_effector_cmd;
ros::Publisher pub_cmd;
ros::Subscriber sub_end_effector_state;
std::string ee_name;

bool wait(ros::Duration _timeout)
{
    ros::Rate r(100);
    ros::Time start = ros::Time::now();

    while(ros::ok())
    {
        ROS_DEBUG("Waiting...");
        if (ros::Time::now() - start > _timeout) { return true; };

        r.sleep();
    }

    return false;
}

bool sendGripperCommand(std::string _cmd, bool _block, double _timeout, std::string _args)
{
    IOComponentCommand ee_cmd;
    ee_cmd.time = ros::Time::now();
    ee_cmd.op = _cmd;
    ee_cmd.args = "";
    if(_args != "")
    {
        ee_cmd.args = _args;
    }

    pub_cmd.publish(ee_cmd);

    if(_block)
    {
        ros::Duration timeout(_timeout);
        return wait(timeout);
    }

    return true;
}

bool open(bool _block, double _timeout)
{
    std::string arg = "{\"signals\": {\"grip_BJech7Hky4\": {\"data\": [true], \"format\": {\"type\": \"bool\"}}}}";
    return sendGripperCommand("set", _block, _timeout, arg);
}

bool close(bool _block, double _timeout)
{
    std::string arg = "{\"signals\": {\"grip_BJech7Hky4\": {\"data\": [false], \"format\": {\"type\": \"bool\"}}}}";
    return sendGripperCommand("set", _block, _timeout, arg);
}

bool stop(bool _block, double _timeout)
{
    std::string arg = "{\"signals\": {\"go\": {\"data\": [false], \"format\": {\"type\": \"bool\"}}}}";
    return sendGripperCommand("set", _block, _timeout, arg);
}

void initialize(double _timeout)
{
    IOComponentCommand cmd;
    cmd.time = ros::Time::now();
    cmd.op = "activate";
    cmd.args = "{\"devices\": [\"" + ee_name + "\"]}";
    pub_end_effector_cmd.publish(cmd);
    ros::Duration(0.5).sleep();
}

void gripperInitCb(const IONodeStatus &msg)
{
    if (ee_name == "")
    {
        if (msg.devices.size() > 0)
        {
            if (ee_name != msg.devices[0].name)
            {
                ee_name = msg.devices[0].name;
                ROS_INFO("Received EE Name: %s", ee_name.c_str());
                pub_cmd = n->advertise<IOComponentCommand>( "/io/end_effector/" + ee_name + "/command", 10);
            }
            if (msg.devices[0].status.tag == "down" || msg.devices[0].status.tag == "unready")
            {
                ROS_INFO("Activating ClickSmart...");
                initialize(0.5);
            }
            sub_end_effector_state.shutdown();
        }
    }
}

void targetCb(const geometry_msgs::Pose& msg)
{
    move_group->setPoseTarget(msg);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group->move();
    close(true, 1.5);
    open(true, 1.5);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hiro_move_group_interface");
    n = std::make_shared<ros::NodeHandle>();
    ee_name = "";

    ros::Subscriber sub_move_target = n->subscribe("/hiro/sawyer/target", 1, targetCb);
    sub_end_effector_state = n->subscribe("/io/end_effector/state", 3, gripperInitCb);
    pub_end_effector_cmd = n->advertise<IOComponentCommand>("/io/end_effector/command", 10);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group->setPlannerId("RRTConnect");

    ROS_INFO("Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group->getEndEffectorLink().c_str());

    geometry_msgs::Pose init_pose;
    init_pose.orientation.x = 0.0;
    init_pose.orientation.y = 1.0;
    init_pose.orientation.z = 0.0;
    init_pose.orientation.w = 0.0;
    init_pose.position.x = 0.45;
    init_pose.position.y = 0.45;
    init_pose.position.z = 0.3;
    move_group->setPoseTarget(init_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group->move();

    ros::waitForShutdown();

    return 0;
}