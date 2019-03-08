#include <localization.hpp>

namespace Turtlebot
{

Localization::Localization(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Localization::laserScanCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Localization::odomCallback, this);
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Localization::mapCallback, this);
    m_pose_icp = new PoseEstimationICP;
    pnh.getParam("num_particles", m_num_particles);
    pnh.getParam("percent_to_drop", m_percent_to_drop);
    pnh.getParam("percent_to_average", m_percent_to_average);
}

Localization::~Localization()
{
    delete m_fake_scan;
    delete m_pose_icp;    
}

void Localization::Localize()
{
    if(!m_have_map || !m_have_scan || !m_have_odom)
    {
        return;
    }
    if(!m_initialized)
    {
        initializeLocalization();
        m_initialized = true;
    }
    std::deque<Particle> particles = sampleParticles();
    takeActionParticles(particles);
    calcParticleWeights(particles);
    m_particles = particles;
    pruneAndNormalizeParticles();
    tf::StampedTransform final_tf = calcFinalTransform();
    m_broad.sendTransform(final_tf);
    setPreviousPose(final_tf);  
}

void Localization::initializeLocalization()
{
    const std::vector<Point> &open_points = getFreeSpace();
    for(int particle_it = 0; particle_it < m_num_particles; particle_it++)
    {
        m_particles.push_back(getRandomParticle(open_points));
    }
}

const std::vector<Point> Localization::getFreeSpace()
{
    std::vector<Point> points;
    for(int point_it = 0; point_it < m_map->data.size(); point_it++)
    {
        if(int(m_map->data[point_it]) == 0)
        {
            points.push_back(getMapCoords(point_it));
        }
    }
    return points;
}

const Particle Localization::getRandomParticle(const std::vector<Point> &open_points)
{
    std::srand(ros::Time::now().toSec());
    const int &rand_sample = std::rand() % open_points.size();
    tf::Pose pose;
    pose.getOrigin().setX(open_points[rand_sample].x);
    pose.getOrigin().setY(open_points[rand_sample].y);
    pose.getOrigin().setZ(0);
    const double &yaw = rand_sample / open_points.size() * 2 * M_PI;
    pose.setRotation(tf::createQuaternionFromYaw(yaw));
    return Particle(pose, 1 / m_num_particles);
}

const Point Localization::getMapCoords(const int &location)
{
    const int &height = m_map->info.height;
    const double &resolution = m_map->info.resolution;
    const double &x = location % height / resolution;
    const double &y = int(location / height) / resolution;
    return Point(x, y);
}

const std::deque<Particle> Localization::sampleParticles()
{
    std::srand(ros::Time::now().toSec());
    std::deque<Particle> sampled_particles;
    for(int sample_id = 0; sample_id < m_num_particles; sample_id++)
    {
        double rand_sample = (std::rand() % 1000) / 1000;
        for(const auto &particle : m_particles)
        {
            rand_sample -= particle.weight;
            if(rand_sample <= 0)
            {
                sampled_particles.push_back(particle);
                break;
            }
        }
    }
    return sampled_particles;
}

void Localization::takeActionParticles(std::deque<Particle> &particles)
{
    for(auto &particle : particles)
    {
        const double &dx = m_odom_at_scan.pose.pose.position.x - m_odom_at_last_scan.pose.pose.position.x;
        const double &dy = m_odom_at_scan.pose.pose.position.y - m_odom_at_last_scan.pose.pose.position.y;
        const double &x = particle.pose.getOrigin().getX() + dx;
        const double &y = particle.pose.getOrigin().getY() + dy;
        particle.pose.setOrigin(tf::Vector3(x, y, 0));
        tf::Quaternion q, last_q;
        q.setW(m_odom_at_scan.pose.pose.orientation.w);
        q.setX(m_odom_at_scan.pose.pose.orientation.x);
        q.setY(m_odom_at_scan.pose.pose.orientation.y);
        q.setZ(m_odom_at_scan.pose.pose.orientation.z);
        last_q.setW(m_odom_at_last_scan.pose.pose.orientation.w);
        last_q.setX(m_odom_at_last_scan.pose.pose.orientation.x);
        last_q.setY(m_odom_at_last_scan.pose.pose.orientation.y);
        last_q.setZ(m_odom_at_last_scan.pose.pose.orientation.z);
        tf::Quaternion dq = q - last_q;
        dq.normalize();
        double qw = particle.pose.getRotation().getW() + dq.getW();
        double qx = particle.pose.getRotation().getX() + dq.getX();
        double qy = particle.pose.getRotation().getY() + dq.getY();
        double qz = particle.pose.getRotation().getZ() + dq.getZ();
        const double &normalizer = std::sqrt(std::pow(qw, 2) + std::pow(qx, 2) + std::pow(qy, 2) + std::pow(qz, 2));
        particle.pose.getRotation().setW(qw / normalizer);
        particle.pose.getRotation().setX(qx / normalizer);
        particle.pose.getRotation().setY(qy / normalizer);
        particle.pose.getRotation().setZ(qz / normalizer);
    }    
}

void Localization::calcParticleWeights(std::deque<Particle> &particles)
{
    const sensor_msgs::LaserScan &fake_scan = m_prev_scan;// m_fake_scan->getFakeScan(m_prev_pose);
    m_prev_scan = *m_scan;
    const tf::Pose &sensor_pose_estimate = m_pose_icp->getTransform(fake_scan, *m_scan);
    for(auto &particle : particles)
    {
        const double &distance_score = calcDistanceScore(particle.pose.getOrigin(), sensor_pose_estimate.getOrigin());
        const double &rotation_score = calcRotationScore(particle.pose.getRotation(), sensor_pose_estimate.getRotation());
        const double &weight = distance_score + rotation_score;
        particle.weight = weight;
    }
    std::sort(particles.begin(), particles.end());    
}

const double Localization::calcDistanceScore(const tf::Point &particle_pt, const tf::Point &sensor_pt)
{
    const double &dx = fabs(particle_pt.getX() - sensor_pt.getX());
    const double &dy = fabs(particle_pt.getY() - sensor_pt.getY());
    const double &dz = fabs(particle_pt.getZ() - sensor_pt.getZ());
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
    if(dist <= 0.001)
    {
        dist = 0.001;
    }
    return 1 / dist;
}

const double Localization::calcRotationScore(const tf::Quaternion &particle_q, const tf::Quaternion &sensor_q)
{
    tf::Quaternion dq = particle_q - sensor_q;
    dq.normalize();
    double roll, pitch, yaw;
    tf::Matrix3x3(dq).getRPY(roll, pitch, yaw);
    if(fabs(yaw) <= 0.01)
    {
        yaw = 0.01;
    }
    return 1 / fabs(yaw);
}

void Localization::pruneAndNormalizeParticles()
{
    const int &num_particles_to_prune = m_particles.size() * (m_percent_to_drop / 100);
    for(int i = 0; i < num_particles_to_prune; i++)
    {
        m_particles.pop_front();
    }
    double total_weight = 0;
    for(const auto &particle : m_particles)
    {
        total_weight += particle.weight;
    }
    for(auto &particle : m_particles)
    {
        particle.weight = particle.weight / total_weight;
    }
}

const tf::StampedTransform Localization::calcFinalTransform()
{
    const int &num_particles_to_average = m_num_particles * (m_percent_to_average / 100);
    const int &start_it = m_particles.size() - num_particles_to_average - 1;
    double x = 0;
    double y = 0;
    double yaw = 0;
    for(int  particle_it = start_it;  particle_it < m_particles.size() - 1; particle_it++)
    {
        x += m_particles[particle_it].pose.getOrigin().getX();
        y += m_particles[particle_it].pose.getOrigin().getY();
        double roll, pitch, yaw_;
        tf::Matrix3x3(m_particles[particle_it].pose.getRotation()).getRPY(roll, pitch, yaw_);
        yaw += yaw_;        
    }
    x = x / num_particles_to_average;
    y = y / num_particles_to_average;
    yaw = yaw / num_particles_to_average;
    tf::StampedTransform transform;
    transform.stamp_ = ros::Time::now();
    transform.child_frame_id_ = "/base_link";
    transform.frame_id_ = "/map";
    transform.setOrigin(tf::Vector3(x, y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(yaw));
    return transform;
}

void Localization::integratePoseToCurrentTime()
{

}

void Localization::setPreviousPose(const tf::StampedTransform &transform)
{
    m_prev_pose.position.x = transform.getOrigin().getX();
    m_prev_pose.position.y = transform.getOrigin().getY();
    m_prev_pose.position.z = transform.getOrigin().getZ();
    m_prev_pose.orientation.w = transform.getRotation().getW();
    m_prev_pose.orientation.x = transform.getRotation().getX();
    m_prev_pose.orientation.y = transform.getRotation().getY();
    m_prev_pose.orientation.z = transform.getRotation().getZ();
    m_odom_at_last_scan = m_odom_at_scan;
}

void Localization::integrateOdomToScanTime()
{
    ros::Duration dt = m_scan->header.stamp - m_odom->header.stamp;
    m_odom_at_scan.header.stamp = m_scan->header.stamp;
    const double &acc_x = (m_prev_odom->twist.twist.linear.x - m_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_prev_odom->twist.twist.linear.y - m_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_prev_odom->twist.twist.angular.z - m_odom->twist.twist.angular.z) / dt.toSec();
    m_odom_at_scan.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt.toSec();
    m_odom_at_scan.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt.toSec();
    m_odom_at_scan.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    q.setW(m_odom->pose.pose.orientation.w);
    q.setX(m_odom->pose.pose.orientation.x);
    q.setY(m_odom->pose.pose.orientation.y);
    q.setZ(m_odom->pose.pose.orientation.z);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const double &radius_curvature = std::pow(m_odom_at_scan.twist.twist.linear.x, 2) +
                                     std::pow(m_odom_at_scan.twist.twist.linear.y, 2) /
                                     m_odom_at_scan.twist.twist.angular.z;
    const double &yaw_f = m_odom_at_scan.twist.twist.linear.z * dt.toSec();
    double x_f;
    double y_f;
    if(std::isnan(radius_curvature))
    {
        x_f = m_odom->pose.pose.position.x + m_odom->twist.twist.linear.x * dt.toSec() + acc_x * std::pow(dt.toSec(), 2) / 2;
        y_f = m_odom->pose.pose.position.y + m_odom->twist.twist.linear.y * dt.toSec() + acc_y * std::pow(dt.toSec(), 2) / 2;
    }
    else
    {
        x_f = m_odom->pose.pose.position.x + radius_curvature * cos(yaw_f) * cos(yaw);
        y_f = m_odom->pose.pose.position.y + radius_curvature * sin(yaw_f) * sin(yaw);
    }
    const tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    m_odom_at_scan.pose.pose.position.x = x_f;
    m_odom_at_scan.pose.pose.position.y = y_f;
    m_odom_at_scan.pose.pose.orientation.w = q_f.getW();
    m_odom_at_scan.pose.pose.orientation.x = q_f.getX();
    m_odom_at_scan.pose.pose.orientation.y = q_f.getY();
    m_odom_at_scan.pose.pose.orientation.z = q_f.getZ();
    ROS_INFO_STREAM(x_f << " " << y_f);
}

void Localization::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{    
    if(!m_have_scan && m_have_odom)
    {        
        m_have_scan = true;
        m_prev_scan = *msg;
        m_odom_at_last_scan = *m_odom;
    }
    m_scan = msg;
    if(m_have_odom)
    {
        integrateOdomToScanTime();
    }
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_prev_odom = msg;
    }
    m_odom = msg;
}

void Localization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_map)
    {
        m_have_map = true;
        m_fake_scan = new FakeScan(*msg);
        m_map = msg;
    }
}

}
