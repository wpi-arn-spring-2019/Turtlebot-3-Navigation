#include <localization.hpp>

namespace Turtlebot
{

Localization::Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Localization::laserScanCallback, this);
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Localization::mapCallback, this);
    m_pose_icp = new PoseEstimationICP;
}

Localization::~Localization()
{

}

void Localization::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    m_scan = msg;
}

void Localization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_map)
    {
        m_have_map = true;
        m_map = msg;
    }
}

}
