#include <fake_scan.hpp>

namespace Turtlebot
{

const sensor_msgs::LaserScan FakeScan::getFakeScan(const geometry_msgs::Pose &pose)
{
    sensor_msgs::LaserScan scan = m_scan;
    scan.ranges.clear();
    for(float inc = m_scan.angle_min; inc < m_scan.angle_max; inc+= m_scan.angle_increment)
    {

        const double &dist = laserThrower(pose, inc);
        writeScan(dist, scan);

    }
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "fake_laser";
    return scan;
}

void FakeScan::writeScan(const double &dist, sensor_msgs::LaserScan &scan)
{
    scan.ranges.push_back(dist);

    ////////////////////////////////////////////////
//    Calculate the euclidian distance between obstacle x,y and pose x,y and multiply it with resoution to find range in metere for given
//    angle of scan
}

const int FakeScan::getLocation(const Point &pt)
{
    const double &resolution = m_map.info.resolution;
    const double &height = m_map.info.height;
    const double &origin_x = m_map.info.origin.position.x;
    const double &origin_y = m_map.info.origin.position.y;
    const int &x = (pt.x - origin_x) / resolution;
    const int &y = (pt.y - origin_y) / resolution;
    return x + y * height;
}

double FakeScan::laserThrower(const geometry_msgs::Pose &pose, const float &inc)
{

    //////////////////////////////////////////////////////////////////////////////////
    //   modify this quaternion
    tf::Quaternion q;
    q.setW(pose.orientation.w);
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double ray_dist = 0;   // find the minimum ray_dist and put it instead of zero
    /////////////////////////////////////////////////////////////////////////////////////////

    double angle = yaw + inc;

    const double &x = pose.position.x + ray_dist * cos(angle);
    const double &y = pose.position.y + ray_dist * sin(angle);
    Point pt(x, y);

    while(int(m_map.data[getLocation(pt)]) != 100)
    {
        pt.x = pose.position.x + ray_dist * cos(angle);
        pt.y = pose.position.y + ray_dist * sin(angle);
        ray_dist += 0.01;
        if(ray_dist > m_scan.range_max)
        {
            return std::numeric_limits<double>::infinity();
        }
    }
    if(ray_dist < m_scan.range_min)
    {
        return m_scan.range_min;
    }
    return ray_dist;
}


}
