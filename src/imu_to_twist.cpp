#include <twist_calculator/imu_to_twist.h>

namespace twist_calculator
{
    ImuToTwist::ImuToTwist(ros::NodeHandle nh,ros::NodeHandle pnh) : listener_(buffer_)
    {
        buffer_.setUsingDedicatedThread(true);
        nh_ = nh;
        pnh_ = pnh;
        pnh_.param<bool>("publish_timestamp", publish_timestamp_, false);
        pnh_.param<bool>("enable_twist_reset", enable_twist_reset_, true);
        pnh_.param<std::string>("robot_frame", robot_frame_, "/base_link");
        pnh_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
        pnh_.param<std::string>("curretn_twist_topic", curretn_twist_topic_, "/curretn_twist");
        curretn_twist_.linear.x = 0.0;
        curretn_twist_.linear.y = 0.0;
        curretn_twist_.linear.z = 0.0;
        curretn_twist_.angular.x = 0.0;
        curretn_twist_.angular.y = 0.0;
        curretn_twist_.angular.z = 0.0;
        dynaparam_callback_func_ = boost::bind(&ImuToTwist::dynaparamCallback, this, _1, _2);
        dynaparam_server_.setCallback(dynaparam_callback_func_);
        if(publish_timestamp_)
        {
            twist_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("twist",1);
        }
        else
        {
            twist_pub_ = pnh_.advertise<geometry_msgs::Twist>("twist",1);
        }
        imu_sub_ = nh_.subscribe(imu_topic_,1,&ImuToTwist::imuCallback,this);
        if(enable_twist_reset_)
        {
            curretn_twist_sub_ = nh_.subscribe(curretn_twist_topic_,1,&ImuToTwist::currentTwistCallback,this);
        }
    }

    ImuToTwist::~ImuToTwist()
    {

    }

    void ImuToTwist::dynaparamCallback(twist_calculator::ImuToTwistConfig &config, uint32_t level)
    {
        config_ = config;
        return;
    }

    void ImuToTwist::currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
    {
        curretn_twist_ = msg->twist;
        return;
    }

    void ImuToTwist::imuCallback(const sensor_msgs::Imu::ConstPtr msg)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = buffer_.lookupTransform(robot_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        geometry_msgs::Vector3Stamped acc_vec;
        acc_vec.header = msg->header;
        acc_vec.vector = msg->linear_acceleration;
        tf2::doTransform(acc_vec,acc_vec,transform_stamped);
        ros::Time now = ros::Time::now();
        if(stamp_)
        {
            curretn_twist_.linear.x = curretn_twist_.linear.x + (now-stamp_.get()).toSec()*(acc_vec.vector.x-config_.acceralation_offset_x);
            curretn_twist_.linear.y = curretn_twist_.linear.y + (now-stamp_.get()).toSec()*(acc_vec.vector.y-config_.acceralation_offset_y);
            curretn_twist_.linear.z = curretn_twist_.linear.z + (now-stamp_.get()).toSec()*(acc_vec.vector.z-config_.acceralation_offset_z-gravitational_acceleration);
            geometry_msgs::Vector3Stamped ang_vel_vec;
            ang_vel_vec.header = msg->header;
            ang_vel_vec.vector = msg->angular_velocity;
            tf2::doTransform(ang_vel_vec,ang_vel_vec,transform_stamped);
            curretn_twist_.angular = ang_vel_vec.vector;
            stamp_ = now;
            if(publish_timestamp_)
            {
                geometry_msgs::TwistStamped twist_stamped;
                twist_stamped.header = msg->header;
                twist_stamped.header.stamp = now;
                twist_stamped.header.frame_id = robot_frame_;
                twist_stamped.twist = curretn_twist_;
                twist_pub_.publish(twist_stamped);
            }
            else
            {
                twist_pub_.publish(curretn_twist_);
            }
        }
        else
        {
            stamp_ = now;
        }
        return;
    }
}