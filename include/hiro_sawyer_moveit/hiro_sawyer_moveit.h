#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <intera_core_msgs/IODeviceStatus.h>
#include <intera_core_msgs/IONodeStatus.h>
#include <intera_core_msgs/IONodeConfiguration.h>
#include <intera_core_msgs/IODeviceConfiguration.h>
#include <intera_core_msgs/IOComponentCommand.h>
#include <intera_core_msgs/IOStatus.h>
#include <intera_core_msgs/JointCommand.h>

#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>

class HiroSawyer
{
private:
    ros::NodeHandle n;
    ros::AsyncSpinner spinner;

    std::string PLANNING_GROUP;
    std::string ee_name;

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher pub_end_effector_cmd;
    ros::Publisher pub_cmd;
    ros::Publisher pub_torque_cmd;
    ros::Subscriber sub_end_effector_state;
    ros::Subscriber sub_joints_state;
    ros::Subscriber sub_move_target;

    std::vector<double> cur_pos;
    std::vector<double> cur_vel;
    std::vector<double> Kp;
    std::vector<double> Kd;
    std::vector<double> effort_limit;
public:
    HiroSawyer(std::string name, std::string group = "right_arm");
    ~HiroSawyer();

    void setK(std::vector<double>& k, double k0, double k1, double k2, double k3, double k4, double k5, double k6);
    bool wait(ros::Duration _timeout);
    bool sendGripperCommand(std::string _cmd, bool _block, double _timeout, std::string _args);
    // gripper functions
    bool open(bool _block, double _timeout);
    bool close(bool _block, double _timeout);
    bool stop(bool _block, double _timeout);
    void initialize(double _timeout);
    // utilities
    bool reached(std::vector<double>& target);
    double norm(std::vector<double>& a, std::vector<double>& b);
    void move(moveit_msgs::RobotTrajectory& traj);
    void gotoPose(geometry_msgs::Pose& target);
    // callbacks
    void targetCb(const geometry_msgs::Pose& msg);
    void gripperInitCb(const intera_core_msgs::IONodeStatus& msg);
    void stateCb(const sensor_msgs::JointState& msg);
};