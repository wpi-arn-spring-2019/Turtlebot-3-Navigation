#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace Turtlebot
{

class OdomCorrection
{
public:
    OdomCorrection(ros::NodeHandle &nh);
    ~OdomCorrection() = default;

    void correctOdom();

private:
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    ros::Subscriber m_pose_sub;
    tf::TransformBroadcaster m_broad;
    tf::TransformListener m_list;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr m_pose;
    bool m_have_pose = false;

};

}
