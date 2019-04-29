#include <hiro_sawyer_moveit/hiro_sawyer_moveit.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hiro_DSM");
    HiroSawyer sawyer("hiro_DSM", "right_arm");
    ros::AsyncSpinner spinner(8);

    geometry_msgs::Pose init_pose;
    init_pose.orientation.x = 0.0;
    init_pose.orientation.y = 1.0;
    init_pose.orientation.z = 0.0;
    init_pose.orientation.w = 0.0;
    init_pose.position.x = 0.45;
    init_pose.position.y = 0.45;
    init_pose.position.z = 0.3;
    sawyer.gotoPose(init_pose);

    ros::waitForShutdown();

    return 0;
}
