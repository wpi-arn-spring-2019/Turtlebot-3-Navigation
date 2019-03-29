#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <turtlebot_msgs/GoalPose.h>

namespace Turtlebot
{

class LocalNavigator
{
public:
    LocalNavigator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LocalNavigator();

    void setWaypoint();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);


    ros::Subscriber m_path_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_map_sub;
    ros::Publisher m_goal_pose_pub;


};

}
