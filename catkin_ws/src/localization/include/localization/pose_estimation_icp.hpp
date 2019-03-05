#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace Turtlebot
{

class PoseEstimationICP
{
public:
    PoseEstimationICP(const sensor_msgs::LaserScan &base_scan, const sensor_msgs::LaserScan::ConstPtr &new_scan);
    ~PoseEstimationICP();

private:


};

}
