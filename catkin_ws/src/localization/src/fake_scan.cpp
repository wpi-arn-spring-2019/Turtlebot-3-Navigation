#include <fake_scan.hpp>

namespace Turtlebot
{

const sensor_msgs::LaserScan FakeScan::getFakeScan(const geometry_msgs::Pose &pose)
{
    ROS_INFO_STREAM(m_map.data[0]);

}

}
