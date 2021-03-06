#include <ros/ros.h>
#include <extended_kalman_filter.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_ekf_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Turtlebot::ExtendedKalmanFilter ekf(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
