#include <cmath>
#include <iostream>

#include "ros/ros.h"
#include "intera_core_msgs/JointLimits.h"   // to get min/max joint angle and min/max torque
#include "sensor_msgs/JointState.h"         // to get current configuration and speed
#include "std_msgs/Float64.h"               // to advertise torques
#include "intera_core_msgs/JointCommand.h"  // to publish torques


bool get_current_jointstates;
bool show_joint_limits;
bool fill_in_desired_endeffector_pose;

// PD control gains
std::vector<double> Kp(7);
std::vector<double> Kd(7);

// joint angle configurations
std::vector<double> q_d(7);             // desired robot configuration [rad]
std::vector<double> q(7);               // current robot configuration [rad]
std::vector<double> q_lower_limit(7);   // lower limit configuration [rad]
std::vector<double> q_upper_limit(7);   // upper limit configuration [rad]

// joint angle velocities
std::vector<double> dotq(7);            // current joint rate [rad/s]

// torques
std::vector<double> tau(7);             // torque to be applied to system [Nm]
std::vector<double> tau_limit(7);       // min/max torque that can be applied [Nm]

// applied reference
std::vector <std::vector<double> > q_v(2, std::vector<double> (7));     // applied joint angles [rad] 2 rows, 7 columns)

// navigation field
std::vector<double> rho(7);             // navigation field [rad/s]
std::vector<double> eta(1);             // smoothing parameter []


// publishers and subscribers
ros::Subscriber sub_limits;
ros::Subscriber sub_state;
ros::Publisher pub_torque_cmd;


void stateCallback(const sensor_msgs::JointState& statemsg)
{
    for (int i=0;i<7;i++)
    {
        q[i] = statemsg.position[i+1];      // i=0: state head_pan
        dotq[i] = statemsg.velocity[i+1];   // i=0: state head_pan
    }
    get_current_jointstates = true;
}


void limitsCallback(const intera_core_msgs::JointLimits& limitsmsg)
{
    for (int i=0;i<7;i++)
    {
        q_lower_limit[i] = limitsmsg.position_lower[i+1];   // i=0: limits head_pan
        q_upper_limit[i] = limitsmsg.position_upper[i+1];   // i=0: limits head_pan
        tau_limit[i] = limitsmsg.effort[i+1];               // i=0: limits head_pan
    }
    if (get_current_jointstates == true && show_joint_limits == false)
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

void vectorsInitialization()
{
    for (int i=0;i<7;i++)
    {
        q_v[0][i] = q[i];
    }
}

// void navigationField()
// {
//     for (int t=0; t<qdlength; t++)
//     {
//         rho = (qd[t,:] - qv)/max(norm(qd[t,:]-qv),eta);


//     }
// }


int main(int argc, char **argv)
{
    get_current_jointstates = false;
    show_joint_limits = false;
    fill_in_desired_endeffector_pose = false;

    /**
    * Initialize ROS, specify ROS node, and initialize this ROS node
    */
    ros::init(argc, argv, "sawyer_control_ERG_v2"); // initialize ROS and specify the node name
    ros::NodeHandle n; // will fully initialize this node
    ros::Rate loop_rate(1000); // run at 1000Hz

    /**
    * Get the current joint angles and joint velocities
    */
    sub_state = n.subscribe("/robot/joint_states",500,stateCallback);
    ros::spinOnce();


    // Control parameters
    Kp[0]=10;//150;//500;
    Kp[1]=10;//400;
    Kp[2]=10;//250;
    Kp[3]=10;//300;
    Kp[4]=5;//300;
    Kp[5]=5;//100;
    Kp[6]=5;

    Kd[0]=1;//25
    Kd[1]=1;//30;
    Kd[2]=1;//30;
    Kd[3]=1;//50;
    Kd[4]=0.5;//8;
    Kd[5]=0.5;//5;
    Kd[6]=0.5;//4


    // /**
    // * Send computed torques to robot
    // * use "torque_control" mode
    // */
    pub_torque_cmd = n.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command",500);

    while (ros::ok())
    {

        /**
        * Get the joint limitations: limits of joint angles and limits of torques
        * USER has to see the following data when running this program in the terminal
        * - joint lower limits
        * - current joint angles
        * - joint upper limits
        */
        if (get_current_jointstates == true && show_joint_limits == false && fill_in_desired_endeffector_pose == false)
        {
            sub_limits = n.subscribe("/robot/joint_limits",1000, limitsCallback); // get q_lower_limit, q_upper_limit, tau_limit
            vectorsInitialization(); // qv[1,:] = q[:]
            std::cout << "q0 = " + std::to_string(q[0]) + " rad" << std::endl;
            std::cout << "qv0 = " + std::to_string(q_v[0][0]) + " rad" << std::endl;
        }


        // /**
        // * User has to give the following data to the program
        // * - xe_d = desired end-effector position
        // */
        // else if (get_current_jointstates == true && show_joint_limits == true && fill_in_desired_endeffector_pose == false)
        // {
        //     std::cout << "Give 7 desired joint angles: ";
        //     for (int i=0; i<q_d.size();i++)
        //     {
        //         double input;
        //         std::cin >> input;
        //         q_d[i] = input;
        //     }
        //     fill_in_desired_endeffector_pose=true;
        // }





        // /**
        // * Explicit Reference Governor
        // */
        // for (int t=0; t<qdlength; t++)
        // {
        //     rho = (qd[t,:] - qv)/max(norm(qd[t,:]-qv),eta);
        // }




        // /**
        // * Calculate the torques that have to be given to the Sawyer robot
        // * For safety reasons, take into account the torque limits
        // * if applied_torque > max_torque --> applied_torque = max_torque
        // * if applied torque < min_torque --> applied_torque = min_torque
        // * applied torque = tau = vector
        // */
        // else if (get_current_jointstates == true && show_joint_limits == true &&  fill_in_desired_endeffector_pose == true)
        // {
        //     sub_state = n.subscribe("/robot/joint_states",500,stateCallback);
        //     std::cout << "q0: " + std::to_string(q[0]) << std::endl;
        //     for(int i=0; i<7; i++)
        //     {
        //         tau[i] = Kp[i]*(q_d[i] - q[i]) - Kd[i]*dotq[i]; // it seems that gravity compensation is already inherently implemented in Sawyer torque control mode
        //         // if(tau[i]>0.9*tau_limit[i])
        //         // {
        //         //     tau[i] = 0.9*tau_limit[i];
        //         // }
        //         // else if (tau[i]<-0.9*tau_limit[i])
        //         // {
        //         //     tau[i]=-0.9*tau_limit[i];
        //         // }
        //     }
        //     std::cout << "tau0: " + std::to_string(tau[0])  << std::endl
        //               << "error0: " + std::to_string(q_d[0] - q[0])  << std::endl << std::endl;


        //     intera_core_msgs::JointCommand msg;
        //     msg.mode = 3; // TORQUE_MODE
        //     msg.names.push_back("right_j0");
        //     msg.names.push_back("right_j1");
        //     msg.names.push_back("right_j2");
        //     msg.names.push_back("right_j3");
        //     msg.names.push_back("right_j4");
        //     msg.names.push_back("right_j5");
        //     msg.names.push_back("right_j6");

        //     for (int i=0;i<7;i++)
        //     {
        //         msg.effort.push_back(tau[i]);
        //     }

        //     pub_torque_cmd.publish(msg);
        //     get_current_jointstates=false;
        // }


        ros::spinOnce();

        loop_rate.sleep(); // sleep for the time remaining to let us hit our 60Hz publish rate
    }
    return 0;

}
