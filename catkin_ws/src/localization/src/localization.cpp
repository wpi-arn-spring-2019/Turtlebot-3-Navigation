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
    if(!m_have_map)
    {
        return;
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
        const double &dx = m_odom_at_scan->pose.pose.position.x - m_odom_at_last_scan->pose.pose.position.x;
        const double &dy = m_odom_at_scan->pose.pose.position.y - m_odom_at_last_scan->pose.pose.position.y;
        const double &x = particle.pose.getOrigin().getX() + dx;
        const double &y = particle.pose.getOrigin().getY() + dy;
        particle.pose.getOrigin().setX(x);
        particle.pose.getOrigin().setY(y);
        tf::Quaternion q, last_q;
        q.setW(m_odom_at_scan->pose.pose.orientation.w);
        q.setX(m_odom_at_scan->pose.pose.orientation.x);
        q.setY(m_odom_at_scan->pose.pose.orientation.y);
        q.setZ(m_odom_at_scan->pose.pose.orientation.z);
        last_q.setW(m_odom_at_last_scan->pose.pose.orientation.w);
        last_q.setX(m_odom_at_last_scan->pose.pose.orientation.x);
        last_q.setY(m_odom_at_last_scan->pose.pose.orientation.y);
        last_q.setZ(m_odom_at_last_scan->pose.pose.orientation.z);
        const tf::Quaternion &dq = q - last_q;
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
    const sensor_msgs::LaserScan &fake_scan = m_fake_scan->getFakeScan(m_prev_pose);
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
    if(dist == 0)
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
    if(yaw == 0)
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
    const int &num_particles_to_average = m_particles.size() * (m_percent_to_average / 100);
    const int &num_pruned_particles = m_particles.size() * (m_percent_to_drop / 100);
    const int &start_it = m_particles.size() - num_pruned_particles - num_particles_to_average - 1;
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

void Localization::setPreviousPose(const tf::StampedTransform &transform)
{
    m_prev_pose.position.x = transform.getOrigin().getX();
    m_prev_pose.position.y = transform.getOrigin().getY();
    m_prev_pose.position.z = transform.getOrigin().getZ();
    m_prev_pose.orientation.w = transform.getRotation().getW();
    m_prev_pose.orientation.x = transform.getRotation().getX();
    m_prev_pose.orientation.y = transform.getRotation().getY();
    m_prev_pose.orientation.z = transform.getRotation().getZ();
}

void Localization::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    m_scan = msg;
    m_odom_at_scan = m_odom;
}

void Localization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
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
