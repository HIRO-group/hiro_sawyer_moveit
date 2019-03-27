#include <cmath>
#include <iostream>

#include "ros/ros.h"
#include "intera_core_msgs/JointLimits.h"   // to get min/max joint angle and min/max torque
#include "sensor_msgs/JointState.h"         // to get current configuration and speed
#include "std_msgs/Float64.h"               // to advertise torques
#include "intera_core_msgs/JointCommand.h"  // to publish torques


bool get_current_jointangles;
bool show_joint_limits;
bool fill_in_desired_joints;


// joint angle configurations
std::vector<double> q_d(7);             // desired robot configuration [rad]
std::vector<double> q(7);               // current robot configuration [rad]
std::vector<double> q_lower_limit(7);   // lower limit configuration [rad]
std::vector<double> q_upper_limit(7);   // upper limit configuration [rad]

// publishers and subscribers
ros::Subscriber sub_limits;
ros::Subscriber sub_state;
ros::Publisher pub_position_cmd;


void stateCallback(const sensor_msgs::JointState& statemsg)
{
    if (show_joint_limits == false && get_current_jointangles == false)
    {
        for (int i=0;i<7;i++)
        {
            q[i]= statemsg.position[i+1]; // i=0: state head_pan
        }
        get_current_jointangles = true;
    }

}


void limitsCallback(const intera_core_msgs::JointLimits& limitsmsg)
{
    for (int i=0;i<7;i++)
    {
        q_lower_limit[i] = limitsmsg.position_lower[i+1]; // i=0: limits head_pan
        q_upper_limit[i] = limitsmsg.position_upper[i+1]; // i=0: limits head_pan
    }

    if (show_joint_limits == false && get_current_jointangles == true)
    {
        std::cout << "q0_low: " + std::to_string(q_lower_limit[0]) + " rad,   q0: " + std::to_string(q[0]) + " rad,   q0_upp: " + std::to_string(q_upper_limit[0])  + " rad"  << std::endl
                  << "q1_low: " + std::to_string(q_lower_limit[1]) + " rad,   q1: " + std::to_string(q[1]) + " rad,   q1_upp: " + std::to_string(q_upper_limit[1])  + " rad"  << std::endl
                  << "q2_low: " + std::to_string(q_lower_limit[2]) + " rad,   q2: " + std::to_string(q[2]) + " rad,   q2_upp: " + std::to_string(q_upper_limit[2])  + " rad"  << std::endl
                  << "q3_low: " + std::to_string(q_lower_limit[3]) + " rad,   q3: " + std::to_string(q[3]) + " rad,   q3_upp: " + std::to_string(q_upper_limit[3])  + " rad"  << std::endl
                  << "q4_low: " + std::to_string(q_lower_limit[4]) + " rad,   q4: " + std::to_string(q[4]) + " rad,   q4_upp: " + std::to_string(q_upper_limit[4])  + " rad"  << std::endl
                  << "q5_low: " + std::to_string(q_lower_limit[5]) + " rad,   q5: " + std::to_string(q[5]) + " rad,   q5_upp: " + std::to_string(q_upper_limit[5])  + " rad"  << std::endl
                  << "q6_low: " + std::to_string(q_lower_limit[6]) + " rad,   q6: " + std::to_string(q[6]) + " rad,   q6_upp: " + std::to_string(q_upper_limit[6])  + " rad"  << std::endl << std::endl;
        show_joint_limits = true;
    }
}


int main(int argc, char **argv)
{
    get_current_jointangles = false;
    show_joint_limits = false;
    fill_in_desired_joints = false;

    /**
    * Initialize ROS, specify ROS node, and initialize this ROS node
    */
    ros::init(argc, argv, "sawyer_control_qd_posinput"); // initialize ROS and specify the node name
    ros::NodeHandle n; // will fully initialize this node
    ros::Rate loop_rate(60); // run at 60Hz

    /**
    * Get the current joint angles and joint velocities
    */
    sub_state = n.subscribe("/robot/joint_states",60,stateCallback);


    // /**
    // * Send computed torques to robot
    // * use "torque_control" mode
    // */
    pub_position_cmd = n.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command",60);

    while (ros::ok())
    {

        /**
        * Get the joint limitations: limits of joint angles and limits of torques
        * USER has to see the following data when running this program in the terminal
        * - joint lower limits
        * - current joint angles
        * - joint upper limits
        */
        if (get_current_jointangles == true && show_joint_limits == false && fill_in_desired_joints == false)
        {
            sub_limits = n.subscribe("/robot/joint_limits", 60, limitsCallback); // get q_lower_limit, q_upper_limit, tau_limit

        }


        /**
        * User has to give the following data to the program
        * - q_d = desired joint angles (q_d0,q_d1,q_d2,q_d3,q_d4,q_d5,q_d6)
        */
        else if (show_joint_limits == true && fill_in_desired_joints == false)
        {
            std::cout << "Give 7 desired joint angles: ";
            for (int i=0; i<q_d.size();i++)
            {
                double input;
                std::cin >> input;
                q_d[i] = input;
            }
            fill_in_desired_joints=true;
        }


        /**
        * Calculate the torques that have to be given to the Sawyer robot
        * For safety reasons, take into account the torque limits
        * if applied_torque > max_torque --> applied_torque = max_torque
        * if applied torque < min_torque --> applied_torque = min_torque
        * applied torque = tau = vector
        */
        else if (show_joint_limits == true &&  fill_in_desired_joints == true)
        {
            sub_state = n.subscribe("/robot/joint_states",60,stateCallback);

            intera_core_msgs::JointCommand msg;
            msg.mode = 1; // POSITION_MODE
            msg.names.push_back("right_j0");
            msg.names.push_back("right_j1");
            msg.names.push_back("right_j2");
            msg.names.push_back("right_j3");
            msg.names.push_back("right_j4");
            msg.names.push_back("right_j5");
            msg.names.push_back("right_j6");

            for (int i=0;i<7;i++)
            {
                msg.position.push_back(q_d[i]);
            }

            pub_position_cmd.publish(msg);
        }

        // add something to shutdown program
        // abs(q_d[i]-q[i]) < 0.005 --> quit program


        ros::spinOnce();

        loop_rate.sleep(); // sleep for the time remaining to let us hit our 60Hz publish rate
    }
    return 0;

}
