#include <ros/ros.h>
#include <local_navigator.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_navigator_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Turtlebot::LocalNavigator navigator(nh, pnh);

    ros::Rate r(10);

    while(ros::ok)
    {
        ros::spinOnce();

        navigator.setWaypoint();

        r.sleep();
    }

    return 0;

}
