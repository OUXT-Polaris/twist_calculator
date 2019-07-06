// headers in this package
#include <twist_calculator/pose_to_twist.h>

// headers in ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_to_twist_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    twist_calculator::PoseToTwist PoseToTwist(nh,pnh);
    ros::spin();
    return 0;
}