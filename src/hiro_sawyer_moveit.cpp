#include <hiro_sawyer_moveit/hiro_sawyer_moveit.h>
#include <ros/package.h>

#include <string>
#include <cstdlib>
#include <algorithm>

using namespace intera_core_msgs;
using namespace std;
using namespace KDL;

HiroSawyer::HiroSawyer(string name, string group) : n(name), spinner(8), PLANNING_GROUP(group), ee_name(""), move_group(group)
{
    sub_move_target = n.subscribe("/hiro/sawyer/target", 1, &HiroSawyer::targetCb, this);
    sub_end_effector_state = n.subscribe("/io/end_effector/state", 3, &HiroSawyer::gripperInitCb, this);
    sub_joints_state = n.subscribe("/robot/joint_states", 1, &HiroSawyer::stateCb, this);
    pub_end_effector_cmd = n.advertise<IOComponentCommand>("/io/end_effector/command", 10);
    pub_torque_cmd = n.advertise<JointCommand>("/robot/limb/right/joint_command", 1);

    effort_limit = vector<double> {80.0, 80.0, 40.0, 40.0, 9.0, 9.0, 9.0};
    Kp = vector<double> {500, 500, 300, 300, 20, 100, 100};
    Kd = vector<double> {1, 1, 1, 1, 0.5, 0.5, 0.5};

    // create KDL chain
    string path = ros::package::getPath("hiro_sawyer_moveit");
    ROS_INFO("%s", path.c_str());
    string robot_desc_string;
    if (!kdl_parser::treeFromFile(path + "/urdf/sawyer.urdf", kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    if (!kdl_tree.getChain("base", move_group.getEndEffectorLink(), kdl_chain))
    {
        ROS_ERROR("Failed to get kdl chain");
    }
    ROS_INFO("Successfully get KDL Chain with %d joints", kdl_chain.getNrOfJoints());
    joint_num = kdl_chain.getNrOfJoints();
    kdl_cur_pos.resize(joint_num);
    kdl_cur_vel.resize(joint_num);
    kdl_coriolis.resize(joint_num);
    kdl_mass.resize(joint_num);
    KDL::Vector gravity(0.0, 0.0, -9.80);
    dyn_param = make_shared<ChainDynParam>(kdl_chain, gravity);

    coriolis_vector.resize(joint_num);
    mass_matrix.resize(joint_num, joint_num);

    cur_pos = vector<double>(joint_num);
    cur_vel = vector<double>(joint_num);

    spinner.start();

    move_group.setPlannerId("RRTConnect");

    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
}

HiroSawyer::~HiroSawyer()
{
    spinner.stop();
}

bool HiroSawyer::wait(ros::Duration _timeout)
{
    ros::Rate r(200);
    ros::Time start = ros::Time::now();

    while(ros::ok())
    {
        ROS_DEBUG("Waiting...");
        if (ros::Time::now() - start > _timeout) { return true; };

        r.sleep();
    }

    return false;
}

bool HiroSawyer::sendGripperCommand(string _cmd, bool _block, double _timeout, string _args)
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

bool HiroSawyer::open(bool _block, double _timeout)
{
    std::string arg = "{\"signals\": {\"grip_BJech7Hky4\": {\"data\": [true], \"format\": {\"type\": \"bool\"}}}}";
    return sendGripperCommand("set", _block, _timeout, arg);
}

bool HiroSawyer::close(bool _block, double _timeout)
{
    std::string arg = "{\"signals\": {\"grip_BJech7Hky4\": {\"data\": [false], \"format\": {\"type\": \"bool\"}}}}";
    return sendGripperCommand("set", _block, _timeout, arg);
}

bool HiroSawyer::stop(bool _block, double _timeout)
{
    std::string arg = "{\"signals\": {\"go\": {\"data\": [false], \"format\": {\"type\": \"bool\"}}}}";
    return sendGripperCommand("set", _block, _timeout, arg);
}

void HiroSawyer::initialize(double _timeout)
{
    IOComponentCommand cmd;
    cmd.time = ros::Time::now();
    cmd.op = "activate";
    cmd.args = "{\"devices\": [\"" + ee_name + "\"]}";
    pub_end_effector_cmd.publish(cmd);
    ros::Duration(0.5).sleep();
}

void HiroSawyer::gripperInitCb(const IONodeStatus& msg)
{
    if (ee_name == "")
    {
        if (msg.devices.size() > 0)
        {
            if (ee_name != msg.devices[0].name)
            {
                ee_name = msg.devices[0].name;
                ROS_INFO("Received EE Name: %s", ee_name.c_str());
                pub_cmd = n.advertise<IOComponentCommand>( "/io/end_effector/" + ee_name + "/command", 10);
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

void HiroSawyer::stateCb(const sensor_msgs::JointState& msg)
{
    for (int i = 0; i < joint_num; i++)
    {
        cur_pos[i] = msg.position[i+1];   // i=0: state head_pan
        kdl_cur_pos(i) = msg.position[i+1];
        cur_vel[i] = msg.velocity[i+1];   // i=0: state head_pan
        kdl_cur_vel(i) = msg.velocity[i+1];
    }
}

bool HiroSawyer::reached(vector<double>& target)
{
    if (target.size() != joint_num)
    {
        ROS_ERROR("reached size mismatched");
        return false;
    }
    for (int i = 0; i < joint_num; i++)
    {
        if (std::abs(target[i] - cur_pos[i]) > 0.1)
        {
            return false;
        }
    }
    return true;
}

double HiroSawyer::norm(vector<double>& a, vector<double>& b)
{
    if ((a.size() != b.size()) || (a.size() == 0 || b.size() == 0))
    {
        return -1;
    }
    double res = 0;
    for (int i = 0; i < a.size(); i++)
    {
        res += (a[i] - b[i])*(a[i] - b[i]);
    }
    return sqrt(res);
}

void HiroSawyer::move(moveit_msgs::RobotTrajectory& traj)
{
    if (traj.joint_trajectory.points.size() <= 0)
    {
        ROS_ERROR("Invalid trajectory");
        return;
    }
    std::vector<double> tau(joint_num, 0);
    std::vector<double> applied_pos(joint_num, 0);
    std::vector<double> rho(joint_num);
    double delta = 1;
    // init applied pos
    for (int i = 0; i < joint_num; i++)
    {
        applied_pos[i] = cur_pos[i];
    }
    ros::Rate loop_rate(250);
    ros::Time start = ros::Time::now();
    int size = traj.joint_trajectory.points.size();
    for (int p = 0; ros::ok() && p < size; p++)
    {
        std::vector<double> target = traj.joint_trajectory.points[p].positions;
        while (ros::ok() && !reached(target))
        {
            dyn_param->JntToMass(kdl_cur_pos, kdl_mass);
            dyn_param->JntToCoriolis(kdl_cur_pos, kdl_cur_vel, kdl_coriolis);
            double denominator = std::max(norm(target, applied_pos), 0.01);
            for(int i = 0; i < joint_num; i++)
            {
                rho[i] = (target[i] - applied_pos[i])/denominator;
                applied_pos[i] = applied_pos[i] + delta*rho[i]*(ros::Time::now() - start).toSec();
                tau[i] = Kp[i]*(applied_pos[i] - cur_pos[i]) - Kd[i] * cur_vel[i];
            }
            intera_core_msgs::JointCommand msg;
            msg.mode = 3; // TORQUE_MODE
            for (int i = 0; i < joint_num; i++)
            {
                msg.names.push_back("right_j" + std::to_string(i));
                if (std::abs(tau[i]) >= 0.9*effort_limit[i])
                {
                    double sign = tau[i] < 0.0 ? -1.0 : 1.0;
                    msg.effort.push_back(0.9 * effort_limit[i] * sign);
                }
                else
                {
                    msg.effort.push_back(tau[i]);
                }
                // std::cout << msg.names[i] << ": " << msg.effort[i] << std::endl;
            }
            pub_torque_cmd.publish(msg);
            start = ros::Time::now();
            loop_rate.sleep();
        }
    }
}

void HiroSawyer::gotoPose(geometry_msgs::Pose& target)
{
    move_group.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

    // test gripper
    // close(true, 2);
    // open(true, 2);
}

void HiroSawyer::targetCb(const geometry_msgs::Pose& msg)
{
    move_group.setPoseTarget(msg);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    std::cout << my_plan.trajectory_.joint_trajectory.points.size() << std::endl;

    move(my_plan.trajectory_);
}