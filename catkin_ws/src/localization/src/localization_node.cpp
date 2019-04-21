#include <ros/ros.h>
#include <localization.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Turtlebot::Localization loc(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
