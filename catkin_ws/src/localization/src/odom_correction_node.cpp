#include <ros/ros.h>
#include <odom_correction.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_correction_node");

    ros::NodeHandle nh;

    Turtlebot::OdomCorrection oc(nh);

    ros::Rate r(1000);

    while(ros::ok)
    {
        ros::spinOnce();

        oc.correctOdom();

        r.sleep();
    }

    return 0;
}
