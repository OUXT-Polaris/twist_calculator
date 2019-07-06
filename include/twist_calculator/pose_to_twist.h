#ifndef TWIST_CALCULATOR_POSE_TO_TWIST_H_INCLUDED
#define TWIST_CALCULATOR_POSE_TO_TWIST_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace twist_calculator
{
    class PoseToTwist
    {
    public:
        PoseToTwist(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~PoseToTwist();
    private:
        tf2_ros::TransformListener listener_;
        tf2_ros::Buffer buffer_;
        bool publish_timestamp_;
        bool enable_twist_reset_;
        std::string robot_frame_;
        std::string pose_topic_;
        std::string curretn_twist_topic_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
    };
}

#endif  //TWIST_CALCULATOR_POSE_TO_TWIST_H_INCLUDED