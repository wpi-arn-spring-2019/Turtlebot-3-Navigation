#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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
    FakeScan(const nav_msgs::OccupancyGrid &map, const sensor_msgs::LaserScan &scan);
    ~FakeScan(){}

    const sensor_msgs::LaserScan getFakeScan(const geometry_msgs::Pose &pose);

private:
    void convertMatrix();
    double laserThrower(const geometry_msgs::Pose &pose, const float &inc);
    void writeScan(const double &dist, sensor_msgs::LaserScan &scan);
    std::vector<std::vector<int>> m_matrix_map;

    nav_msgs::OccupancyGrid m_map;
    sensor_msgs::LaserScan m_scan;


};

}
