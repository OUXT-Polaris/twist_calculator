// headers in this package
#include <twist_calculator/pose_to_twist.h>

namespace twist_calculator
{
    PoseToTwist::PoseToTwist(ros::NodeHandle nh,ros::NodeHandle pnh)
    {
        nh_ = nh;
        pnh_ = pnh;
        pnh_.param<bool>("publish_timestamp", publish_timestamp_, false);
        pnh_.param<std::string>("robot_frame", robot_frame_, "/base_link");
        pnh_.param<std::string>("pose_topic", pose_topic_, "/robot_pose");
        pose_buffer_ = boost::circular_buffer<geometry_msgs::PoseStamped>(2);
        if(publish_timestamp_)
        {
            twist_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("current_twist",1);
        }
        else
        {
            twist_pub_ = pnh_.advertise<geometry_msgs::Twist>("current_twist",1);
        }
        pose_sub_ = nh_.subscribe(pose_topic_,1,&PoseToTwist::poseCallback,this);
    }

    PoseToTwist::~PoseToTwist()
    {

    }

    void PoseToTwist::poseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        pose_buffer_.push_back(*msg);
        using namespace quaternion_operation;
        if(pose_buffer_.size() != 2)
        {
            return;
        }
        double dt = (pose_buffer_[1].header.stamp-pose_buffer_[0].header.stamp).toSec();
        geometry_msgs::TwistStamped twist;
        twist.header.frame_id = robot_frame_;
        twist.header.stamp = pose_buffer_[1].header.stamp;
        geometry_msgs::Quaternion rot = 
            getRotation(pose_buffer_[0].pose.orientation,
                pose_buffer_[1].pose.orientation);
        twist.twist.angular = convertQuaternionToEulerAngle(rot);
        twist.twist.angular.x = twist.twist.angular.x/dt;
        twist.twist.angular.y = twist.twist.angular.y/dt;
        twist.twist.angular.z = twist.twist.angular.z/dt;
        Eigen::Matrix3d rotation_mat = getRotationMatrix(conjugate(pose_buffer_[0].pose.orientation));
        Eigen::Vector3d vec0,vec1;
        vec0[0] = pose_buffer_[0].pose.position.x;
        vec0[1] = pose_buffer_[0].pose.position.y;
        vec0[2] = pose_buffer_[0].pose.position.z;
        vec1[0] = pose_buffer_[1].pose.position.x;
        vec1[1] = pose_buffer_[1].pose.position.y;
        vec1[2] = pose_buffer_[1].pose.position.z;
        vec0 = rotation_mat*vec0;
        vec1 = rotation_mat*vec1;
        twist.twist.linear.x = (vec1[0]-vec0[0])/dt;
        twist.twist.linear.y = (vec1[1]-vec0[1])/dt;
        twist.twist.linear.z = (vec1[2]-vec0[2])/dt;
        /*
        Eigen::Vector3d trans_vec;
        trans_vec[0] = pose_buffer_[1].pose.position.x-pose_buffer_[0].pose.position.x;
        trans_vec[1] = pose_buffer_[1].pose.position.y-pose_buffer_[0].pose.position.y;
        trans_vec[2] = pose_buffer_[1].pose.position.z-pose_buffer_[0].pose.position.z;
        trans_vec = rotation_mat*trans_vec;
        twist.twist.linear.x = trans_vec[0]/dt;
        twist.twist.linear.y = trans_vec[1]/dt;
        twist.twist.linear.z = trans_vec[2]/dt;
        */
        if(publish_timestamp_)
        {
            twist_pub_.publish(twist);
        }
        else
        {
            twist_pub_.publish(twist.twist);
        }
    }
}