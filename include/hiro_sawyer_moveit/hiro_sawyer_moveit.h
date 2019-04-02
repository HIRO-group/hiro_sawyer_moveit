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

#include <armadillo/armadillo>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
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
    std::vector<double> effort_limit_lower;
    std::vector<double> position_upper;
    std::vector<double> position_lower;

    std::vector<double> tau_pred;
    std::vector<double> q_prev;
    std::vector<double> q_curr;
    std::vector<double> q_dot_prev;
    std::vector<double> q_dot_curr;

    std::vector<double> tau;
    std::vector<double> applied_pos;
    std::vector<double> rho;

    arma::vec q_ddot;
    arma::mat mass_matrix;
    arma::vec part;

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    KDL::JntArray kdl_cur_pos;
    KDL::JntArray kdl_cur_vel;
    KDL::JntArray kdl_coriolis;
    KDL::JntArray kdl_gravity;
    KDL::JntSpaceInertiaMatrix kdl_mass;
    std::shared_ptr<KDL::ChainDynParam> dyn_param;
    unsigned int joint_num;

    void initialize(double _timeout);
    bool reached(std::vector<double>& target);
    double norm(std::vector<double>& a, std::vector<double>& b);
    void updateMass(void);
    bool wait(ros::Duration _timeout);
    bool sendGripperCommand(std::string _cmd, bool _block, double _timeout, std::string _args);
    void updateKDLVectors(std::vector<double>& pos, std::vector<double>& vel);
    double computeDelta(std::vector<double>& t, int sim_times, double sampling_time = 0.001, double delta_tau = 0.05, double kappa_tau = 0.1, double delta_q = 0.05, double kappa_q = 1);
    // callbacks
    void targetCb(const geometry_msgs::Pose& msg);
    void gripperInitCb(const intera_core_msgs::IONodeStatus& msg);
    void stateCb(const sensor_msgs::JointState& msg);
public:
    HiroSawyer(std::string name, std::string group = "right_arm");
    ~HiroSawyer();

    // gripper functions
    bool open(bool _block, double _timeout);
    bool close(bool _block, double _timeout);
    bool stop(bool _block, double _timeout);
    // movement
    void move(moveit_msgs::RobotTrajectory& traj);
    void gotoPose(geometry_msgs::Pose& target);
};