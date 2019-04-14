#include <twist_calculator/imu_to_twist.h>

// headers in ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_to_twist_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    twist_calculator::ImuToTwist ImuToTwist(nh,pnh);
    ros::spin();
    return 0;
}