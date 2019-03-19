#pragma once
#include <ros/ros.h>
#include <turtlebot_msgs/Trajectory.h>


namespace Turtlebot
{

class Controller
{
public:
    Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~Controller();

private:


};

}
