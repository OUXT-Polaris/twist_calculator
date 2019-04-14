#ifndef TWIST_CALCULATOR_IMU_TO_TWIST_H_INCLUDED
#define TWIST_CALCULATOR_IMU_TO_TWIST_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

namespace twist_calculator
{
    class ImuToTwist
    {
    public:
        ImuToTwist(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~ImuToTwist();
    private:
        void currentTwistCallback(const geometry_msgs::Twist::ConstPtr msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr msg);
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber imu_sub_;
        ros::Subscriber curretn_twist_sub_;
        ros::Publisher twist_pub_;
        bool publish_timestamp_;
        std::string imu_topic_;
        std::string curretn_twist_topic_;
        geometry_msgs::Twist curretn_twist_;
    };
}

#endif  //TWIST_CALCULATOR_IMU_TO_TWIST_H_INCLUDED