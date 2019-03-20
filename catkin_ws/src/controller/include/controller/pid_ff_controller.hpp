#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_state.hpp>

namespace Turtlebot
{

class PIDFeedForwardController
{
public:
    PIDFeedForwardController(){}
    ~PIDFeedForwardController() = default;

    const geometry_msgs::Twist getControls(const TurtlebotState &current_state, const TurtlebotState &desired_state);

private:



};

}
