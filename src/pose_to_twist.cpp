// headers in this package
#include <twist_calculator/pose_to_twist.h>

namespace twist_calculator
{
    PoseToTwist::PoseToTwist(ros::NodeHandle nh,ros::NodeHandle pnh) : listener_(buffer_)
    {
        nh_ = nh;
        pnh_ = pnh;
        pnh_.param<bool>("publish_timestamp", publish_timestamp_, false);
        pnh_.param<bool>("enable_twist_reset", enable_twist_reset_, true);
        pnh_.param<std::string>("robot_frame", robot_frame_, "/base_link");
        pnh_.param<std::string>("pose_topic", pose_topic_, "/robot_pose");
        pnh_.param<std::string>("curretn_twist_topic", curretn_twist_topic_, "/curretn_twist");
    }

    PoseToTwist::~PoseToTwist()
    {

    }
}