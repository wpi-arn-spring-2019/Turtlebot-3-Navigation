#include <fake_scan.hpp>

namespace Turtlebot
{

const sensor_msgs::LaserScan FakeScan::getFakeScan(const geometry_msgs::Pose &pose)
{
    pose.position.x;
    tf::Quaternion q;
    q.setW(pose.orientation.w);
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("roll: " << roll << " pitch: " << "yaw: " <<yaw);

}

}
