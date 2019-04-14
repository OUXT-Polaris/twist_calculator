#include <twist_calculator/imu_to_twist.h>

namespace twist_calculator
{
    ImuToTwist::ImuToTwist(ros::NodeHandle nh,ros::NodeHandle pnh)
    {
        nh_ = nh;
        pnh_ = pnh;
        pnh_.param<bool>("publish_timestamp", publish_timestamp_, false);
        pnh_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
        pnh_.param<std::string>("curretn_twist_topic", curretn_twist_topic_, "/curretn_twist");
        curretn_twist_.linear.x = 0.0;
        curretn_twist_.linear.y = 0.0;
        curretn_twist_.linear.z = 0.0;
        curretn_twist_.angular.x = 0.0;
        curretn_twist_.angular.y = 0.0;
        curretn_twist_.angular.z = 0.0;
        if(publish_timestamp_)
        {
            twist_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("twist",1);
        }
        else
        {
            twist_pub_ = pnh_.advertise<geometry_msgs::Twist>("twist",1);
        }
        imu_sub_ = nh_.subscribe(imu_topic_,1,&ImuToTwist::imuCallback,this);
        curretn_twist_sub_ = nh_.subscribe(curretn_twist_topic_,1,&ImuToTwist::currentTwistCallback,this);
    }

    ImuToTwist::~ImuToTwist()
    {

    }

    void ImuToTwist::currentTwistCallback(const geometry_msgs::Twist::ConstPtr msg)
    {
        curretn_twist_ = *msg;
        return;
    }

    void ImuToTwist::imuCallback(const sensor_msgs::Imu::ConstPtr msg)
    {
        curretn_twist_.linear.x = msg->linear_acceleration.x;
        curretn_twist_.linear.y = msg->linear_acceleration.y;
        curretn_twist_.linear.z = msg->linear_acceleration.z;
        curretn_twist_.angular = msg->angular_velocity;
        if(publish_timestamp_)
        {
            geometry_msgs::TwistStamped twist_stamped;
            twist_stamped.header = msg->header;
            twist_stamped.twist = curretn_twist_;
            twist_pub_.publish(twist_stamped);
        }
        else
        {
            twist_pub_.publish(curretn_twist_);
        }
        return;
    }
}