#pragma once
#include <ros/ros.h>
#include <fake_scan.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace Turtlebot
{

class Localization
{
public:
    Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Localization();

private:

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    ros::Subscriber m_scan_sub;
    ros::Subscriber m_map_sub;
    tf::TransformBroadcaster m_broad;


    sensor_msgs::LaserScan::ConstPtr m_scan;

    bool m_have_map = false;
    nav_msgs::OccupancyGrid::ConstPtr m_map;

};

}
