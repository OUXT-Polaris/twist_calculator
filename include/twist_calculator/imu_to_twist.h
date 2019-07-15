#ifndef TWIST_CALCULATOR_IMU_TO_TWIST_H_INCLUDED
#define TWIST_CALCULATOR_IMU_TO_TWIST_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace twist_calculator
{
    constexpr double gravitational_acceleration = 9.80665;
    class ImuToTwist
    {
    public:
        ImuToTwist(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~ImuToTwist();
    private:
        void currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr msg);
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber imu_sub_;
        ros::Subscriber curretn_twist_sub_;
        ros::Publisher twist_pub_;
        bool publish_timestamp_;
        bool enable_twist_reset_;
        boost::optional<ros::Time> stamp_;
        std::string robot_frame_;
        std::string imu_topic_;
        std::string curretn_twist_topic_;
        geometry_msgs::Twist curretn_twist_;
        tf2_ros::TransformListener listener_;
        tf2_ros::Buffer buffer_;
    };
}

#endif  //TWIST_CALCULATOR_IMU_TO_TWIST_H_INCLUDED