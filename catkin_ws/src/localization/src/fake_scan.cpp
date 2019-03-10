#include <fake_scan.hpp>

namespace Turtlebot
{
FakeScan::FakeScan(const nav_msgs::OccupancyGrid &map,  const sensor_msgs::LaserScan &scan) : m_map(map) , m_scan(scan)
{
    m_matrix_map.resize(m_map.info.height);
    for(int a = 0; a < m_map.info.height; a++)
    {
        m_matrix_map[a].resize(m_map.info.width);
    }
    convertMatrix();
}


const sensor_msgs::LaserScan FakeScan::getFakeScan(const geometry_msgs::Pose &pose)
{
    sensor_msgs::LaserScan scan = m_scan;
    scan.ranges.clear();
    for(float inc = m_scan.angle_min; inc <= m_scan.angle_max; inc+= m_scan.angle_increment)
    {

        const double &dist = laserThrower(pose, inc);
        writeScan(dist, scan);

    }
    scan.header.stamp = ros::Time::now();
    return scan;
}

void FakeScan::writeScan(const double &dist, sensor_msgs::LaserScan &scan)
{
    scan.ranges.push_back(dist);

    ////////////////////////////////////////////////
//    Calculate the euclidian distance between obstacle x,y and pose x,y and multiply it with resoution to find range in metere for given
//    angle of scan
}

void FakeScan::convertMatrix()
{
    int location = 0;
    const int &height = m_map.info.height;
    for(const auto &val : m_map.data)
    {
        const int &x = int(location % height);
        const int &y = int(location / height);
        m_matrix_map[x][y] = int(val);
        location++;
    }
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
    double ray_dist = m_scan.range_min;   // find the minimum ray_dist and put it instead of zero
    /////////////////////////////////////////////////////////////////////////////////////////

    int  x = 0 , y = 0;
    double angle = yaw + inc;

    while(m_matrix_map[x][y] != 100) {
        x = int(m_map.info.height / 2  + (pose.position.x + ray_dist * cos(angle)) / m_map.info.resolution);
        y = int(m_map.info.width / 2  + (pose.position.y + ray_dist * sin(angle)) / m_map.info.resolution);
        ray_dist += 0.05;
        if(ray_dist > m_scan.range_max)
        {
            return std::numeric_limits<double>::infinity();
        }
    }
    return ray_dist;
}


}
