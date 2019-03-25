#include <ros/ros.h>
#include <controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    const double &rate = 100;

    ros::Rate r(rate);

    Turtlebot::Controller cont(nh, pnh, rate);

    while(ros::ok)
    {
        ros::spinOnce();

        cont.control();

        r.sleep();
    }

    return 0;

}
