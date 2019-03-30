#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <point.hpp>

namespace Turtlebot
{

class FakeScan
{
public:
    FakeScan(){}
    FakeScan(const nav_msgs::OccupancyGrid &map,const sensor_msgs::LaserScan &scan) : m_map(map), m_scan(scan){}
    ~FakeScan(){}

    sensor_msgs::LaserScan getFakeScan(const geometry_msgs::PoseWithCovarianceStamped &pose);
    nav_msgs::OccupancyGrid m_map;
    sensor_msgs::LaserScan m_scan;
private:
    void convertMatrix();
    const double laserThrower(const geometry_msgs::PoseWithCovarianceStamped &pose, const float &inc) const;
    const int getLocation(const Point &pt) const;
    void writeScan(const double &dist, sensor_msgs::LaserScan &scan);
    std::vector<std::vector<int>> m_matrix_map;




};

}
