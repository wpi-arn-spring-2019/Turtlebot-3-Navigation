#include <ros/ros.h>
#include <odom_correction.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_correction_node");

    ros::NodeHandle nh;

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
