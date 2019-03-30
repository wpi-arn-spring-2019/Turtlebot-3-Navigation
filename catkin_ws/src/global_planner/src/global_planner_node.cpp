#include <ros/ros.h>
#include <global_planner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Turtlebot::GlobalPlanner planner(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
