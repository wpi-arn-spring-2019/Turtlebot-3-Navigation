#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace Turtlebot
{

class OdomCorrection
{
public:
    OdomCorrection(ros::NodeHandle &nh);
    ~OdomCorrection() = default;


private:

};

}
