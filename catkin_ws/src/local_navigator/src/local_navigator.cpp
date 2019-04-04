#include <local_navigator.hpp>

namespace Turtlebot
{

LocalNavigator::LocalNavigator(ros::NodeHandle&nh, ros::NodeHandle&pnh)
{
    m_path_sub = nh.subscribe<nav_msgs::Path>("/global_path", 10, &LocalNavigator::pathCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose", 10, &LocalNavigator::poseCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &LocalNavigator::odomCallback, this);
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &LocalNavigator::costmapCallback, this);
    m_goal_pose_pub = nh.advertise<turtlebot_msgs::GoalPose>("/local_goal_pose", 10);
    m_goal_reached_pub = nh.advertise<std_msgs::Bool>("/goal_reached", 10);
    initializeNavigator(pnh);
}

LocalNavigator::~LocalNavigator()
{

}

void LocalNavigator::initializeNavigator(ros::NodeHandle &pnh)
{
    pnh.getParam("collision_buffer_distance", m_collision_buffer_distance);
    pnh.getParam("waypoint_dist", m_waypoint_dist);
    pnh.getParam("corridor_radius", m_corridor_radius);
    pnh.getParam("max_speed", m_max_speed);
    pnh.getParam("max_accel", m_max_accel);
    pnh.getParam("goal_pos_tol", m_goal_pos_tol);
    pnh.getParam("goal_ang_tol", m_goal_ang_tol);
    pnh.getParam("goal_vel_tol", m_goal_vel_tol);
    pnh.getParam("turtlebot_radius", m_radius);
    pnh.getParam("angular_col_res", m_angular_col_res);
    pnh.getParam("radial_col_res", m_radial_col_res);
    setupCollision();
}

void LocalNavigator::setupCollision()
{
    const int &num_points_radial = (m_radius + m_collision_buffer_distance) / m_radial_col_res;
    const int &num_points_angular = 2 * M_PI / m_angular_col_res;
    for(int i = 0; i < num_points_radial; i++)
    {
        for(int j = 0; j < num_points_angular; j++)
        {
            const double &x = i * m_radial_col_res * cos(j * m_angular_col_res);
            const double &y = i * m_radial_col_res * sin(j * m_angular_col_res);
            tf::Point pt;
            pt.setX(x);
            pt.setY(y);
            pt.setZ(0);
            m_collision_pts.push_back(pt);
        }
    }
}

void LocalNavigator::setWaypoint()
{
    if(m_have_path && m_have_pose && m_have_odom && m_have_map)
    {
        m_current_time = ros::Time::now();
        integrateOdomToCurrentTime();
        integratePoseToCurrentTime();
        if(checkForGoal())
        {
            publishGoalStatus(true);
            return;
        }
        publishGoalPose(calcNextWaypoint());
        publishGoalStatus(false);
    }
}

const bool LocalNavigator::checkForGoal()
{
    const double &dx = fabs(m_path->poses.back().pose.position.x - m_pose_at_cur_time.pose.pose.position.x);
    const double &dy = fabs(m_path->poses.back().pose.position.y - m_pose_at_cur_time.pose.pose.position.y);
    const double &dp = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_path->poses.back().pose.orientation, q);
    const double &goal_th = tf::getYaw(q);
    tf::quaternionMsgToTF(m_pose_at_cur_time.pose.pose.orientation, q);
    const double &cur_th = tf::getYaw(q);
    const double &dth = fabs(goal_th - cur_th);
    const double &dv = std::sqrt(std::pow(m_odom_at_cur_time.twist.twist.linear.x, 2) + std::pow(m_odom_at_cur_time.twist.twist.linear.y, 2));
    return dp < m_goal_pos_tol && dth < m_goal_ang_tol && dv < m_goal_vel_tol;
}

const turtlebot_msgs::GoalPose LocalNavigator::calcNextWaypoint()
{
    int path_id_offset = 0;
    tf::Pose pose = calcGoalPose(path_id_offset);
    while(checkForCollision(calcPointTransform(pose)))
    {
        path_id_offset += 1;
        pose = calcGoalPose(path_id_offset);
    }
    turtlebot_msgs::GoalPose waypoint;
    waypoint.x = pose.getOrigin().getX();
    waypoint.y = pose.getOrigin().getY();
    waypoint.heading = tf::getYaw(pose.getRotation());
    waypoint.speed = calcVelocityGoal(pose);
    waypoint.max_speed = m_max_speed;
    waypoint.max_accel = m_max_accel;
    return waypoint;
}

const tf::Pose LocalNavigator::calcGoalPose(const int &path_it_offset)
{
    int path_it = calcNearestPathPoint() + path_it_offset;
    double x = m_path->poses[path_it].pose.position.x;
    double y = m_path->poses[path_it].pose.position.y;
    double dist_traveled = 0;
    while(dist_traveled < m_waypoint_dist && path_it < m_path->poses.size() - 1)
    {
        x = m_path->poses[path_it].pose.position.x;
        y = m_path->poses[path_it].pose.position.y;
        const double &next_x = m_path->poses[path_it + 1].pose.position.x;
        const double &next_y = m_path->poses[path_it + 1].pose.position.y;
        const double &dx = next_x - x;
        const double &dy = next_y - y;
        dist_traveled += std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        path_it += 1;
    }
    double heading;
    if(path_it < m_path->poses.size() - 1)
    {
        const double &next_x = m_path->poses[path_it + 1].pose.position.x;
        const double &next_y = m_path->poses[path_it + 1].pose.position.y;
        heading = std::atan2(next_y - y, next_x - x);
    }
    else
    {
        tf::Quaternion q;
        tf::quaternionMsgToTF(m_path->poses.back().pose.orientation, q);
        heading = tf::getYaw(q);
    }
    tf::Pose pose;
    pose.setOrigin(tf::Vector3(x, y, 0));
    pose.setRotation(tf::createQuaternionFromYaw(heading));
    return pose;
}

const int LocalNavigator::calcNearestPathPoint()
{
    double shortest_dist = std::numeric_limits<double>::infinity();
    int nearest_point_id;
    for(int path_it = 0; path_it < m_path->poses.size(); path_it++)
    {
        const double &dx = m_path->poses[path_it].pose.position.x - m_pose_at_cur_time.pose.pose.position.x;
        const double &dy = m_path->poses[path_it].pose.position.y - m_pose_at_cur_time.pose.pose.position.y;
        const double &dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(dist < shortest_dist)
        {
            shortest_dist = dist;
            nearest_point_id = path_it;
        }
    }
    return nearest_point_id;
}

const double LocalNavigator::calcVelocityGoal(const tf::Pose &pose)
{
    const double &dx = m_path->poses.back().pose.position.x - pose.getOrigin().getX();
    const double &dy = m_path->poses.back().pose.position.y - pose.getOrigin().getY();
    const double &dist_to_goal = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    const double &time_to_goal = dist_to_goal / m_max_speed;
    const double &time_to_decel = m_max_speed / m_max_accel;
    if(time_to_decel > time_to_goal)
    {
        return 0;
    }
    return m_max_speed;
}

const bool LocalNavigator::checkForCollision(const tf::Transform &transform) const
{
    for(const auto &pt : m_collision_pts)
    {
        const tf::Point &t_pt = transform * pt;
        if(checkPointForCollision(t_pt))
        {
           return true;
        }
    }
    return false;
}

const bool LocalNavigator::checkPointForCollision(const tf::Point &pt) const
{
    return int(m_map->data[calcGridLocation(Point<double>(pt.getX(), pt.getY()))]) == 100;
}

const tf::Transform LocalNavigator::calcPointTransform(const tf::Pose &pose) const
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.getOrigin().getX(), pose.getOrigin().getY(), 0));
    transform.setRotation(pose.getRotation());
    return transform;
}

const int &LocalNavigator::calcGridLocation(const Point<double> &pt) const
{
    const double &resolution = m_map->info.resolution;
    const double &height = m_map->info.height;
    const double &origin_x = m_map->info.origin.position.x;
    const double &origin_y = m_map->info.origin.position.y;
    const int &x = (pt.x - origin_x) / resolution;
    const int &y = (pt.y - origin_y) / resolution;
    const int &location = x + y * height;
    return location;
}

void LocalNavigator::integrateOdomToCurrentTime()
{
    const double &dt = ros::Duration(m_current_time - m_odom->header.stamp).toSec();
    m_odom_at_cur_time.header.stamp = m_current_time;
    double acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt;
    double acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt;
    double acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt;
    if(std::isnan(acc_x) || std::isinf(acc_x))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_y) || std::isinf(acc_y))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_ang) || std::isinf(acc_ang))
    {
        acc_ang = 0;
    }
    m_odom_at_cur_time.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt;
    m_odom_at_cur_time.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt;
    m_odom_at_cur_time.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_cur_time.twist.twist.linear.z * dt;
    const double &x_f = m_odom->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt + acc_x * std::pow(dt, 2) / 2) * cos(yaw);
    const double &y_f = m_odom->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt + acc_y * std::pow(dt, 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_odom_at_cur_time.pose.pose.position.x = x_f;
    m_odom_at_cur_time.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_odom_at_cur_time.pose.pose.orientation = q_;
}

void LocalNavigator::integratePoseToCurrentTime()
{
    const double &dt = ros::Duration(m_current_time - m_pose->header.stamp).toSec();
    double acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt;
    double acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt;
    double acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt;
    if(std::isnan(acc_x) || std::isinf(acc_x))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_y) || std::isinf(acc_y))
    {
        acc_x = 0;
    }
    if(std::isnan(acc_ang) || std::isinf(acc_ang))
    {
        acc_ang = 0;
    }
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom->twist.twist.linear.z * dt;
    const double &x_f = m_pose->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt + acc_x * std::pow(dt, 2) / 2) * cos(yaw);
    const double &y_f = m_pose->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt + acc_y * std::pow(dt, 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_pose_at_cur_time.pose.pose.position.x = x_f;
    m_pose_at_cur_time.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_pose_at_cur_time.pose.pose.orientation = q_;
}

void LocalNavigator::publishGoalPose(const turtlebot_msgs::GoalPose &pose)
{
    m_goal_pose_pub.publish(pose);
}

void LocalNavigator::publishGoalStatus(const bool &reached)
{
    std_msgs::Bool status;
    status.data = reached;
    m_goal_reached_pub.publish(status);
}

void LocalNavigator::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if(!m_have_path)
    {
        m_have_path = true;
    }
    m_path = msg;
}

void LocalNavigator::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose)
    {
        m_have_pose = true;
    }
    m_pose = msg;
}

void LocalNavigator::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_odom = msg;
    }
    m_prev_odom = m_odom;
    m_odom = msg;
}

void LocalNavigator::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_map)
    {
        m_have_map = true;
    }
    m_map = msg;
}


}
