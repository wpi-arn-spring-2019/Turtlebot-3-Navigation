#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <localization.hpp>

namespace Turtlebot
{

class FakeScan
{
public:
    FakeScan(const nav_msgs::OccupancyGrid &map) : m_map(map){}
    ~FakeScan(){}

    const sensor_msgs::LaserScan getFakeScan(const geometry_msgs::Pose &pose);

private:


    nav_msgs::OccupancyGrid m_map;
//    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
//    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

//    ros::Subscriber m_scan_sub;
//    ros::Subscriber m_map_sub;
//    tf::TransformBroadcaster m_broad;


//    sensor_msgs::LaserScan::ConstPtr m_scan;

//    bool m_have_map = false;
//    nav_msgs::OccupancyGrid::ConstPtr m_map;


};

}
