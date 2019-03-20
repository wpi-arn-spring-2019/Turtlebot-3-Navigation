#include <ros/ros.h>
#include <controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    ros::Rate r(100);  

    Turtlebot::Controller cont(nh, pnh);

    while(ros::ok)
    {
        ros::spinOnce();

        cont.control();

        r.sleep();
    }

    return 0;

}
