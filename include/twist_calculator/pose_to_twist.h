#ifndef TWIST_CALCULATOR_POSE_TO_TWIST_H_INCLUDED
#define TWIST_CALCULATOR_POSE_TO_TWIST_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <quaternion_operation/quaternion_operation.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in boost
#include <boost/circular_buffer.hpp>

namespace twist_calculator
{
    class PoseToTwist
    {
    public:
        PoseToTwist(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~PoseToTwist();
    private:
        bool publish_timestamp_;
        std::string robot_frame_;
        std::string map_frame_;
        std::string pose_topic_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher twist_pub_;
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
        ros::Subscriber pose_sub_;
        boost::circular_buffer<geometry_msgs::PoseStamped> pose_buffer_;
        tf2_ros::TransformListener listener_;
        tf2_ros::Buffer buffer_;
    };
}

#endif  //TWIST_CALCULATOR_POSE_TO_TWIST_H_INCLUDED