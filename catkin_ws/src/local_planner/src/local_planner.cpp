#include <local_planner.hpp>

namespace Turtlebot
{

LocalPlanner::LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &LocalPlanner::costmapCallback, this);
    m_goal_pose_sub = nh.subscribe<turtlebot_msgs::GoalPose>("/local_goal_pose", 10, &LocalPlanner::goalPoseCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose", 10, &LocalPlanner::poseCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &LocalPlanner::odomCallback, this);
    m_trajectory_pub = nh.advertise<turtlebot_msgs::Trajectory>("/local_trajectory", 10);
    m_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 10);
    m_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 100);
    m_health_pub = nh.advertise<std_msgs::Bool>("/local_planner_health", 10);
    getParams(pnh);
}

LocalPlanner::~LocalPlanner(){}

void LocalPlanner::getParams(ros::NodeHandle &pnh)
{
      pnh.getParam("collision_buffer_distance", m_collision_buffer_distance);
      pnh.getParam("time_step_ms", m_time_step_ms);
      pnh.getParam("velocity_res", m_velocity_res);
      pnh.getParam("yaw_rate_res", m_yaw_rate_res);
      pnh.getParam("max_yaw_rate", m_max_yaw_rate);
      pnh.getParam("spline_order", m_spline_order);
      pnh.getParam("goal_pos_tolerance", m_goal_pos_tolerance);
      pnh.getParam("goal_heading_tolerance", m_goal_heading_tolerance);
      pnh.getParam("goal_speed_tolerance", m_goal_speed_tolerance);
      pnh.getParam("timeout_ms", m_timeout_ms);
      pnh.getParam("num_retries", m_num_retries);
      pnh.getParam("turtlebot_radius", m_radius);
      pnh.getParam("angular_col_res", m_angular_col_res);
      pnh.getParam("radial_col_res", m_radial_col_res);
}

void LocalPlanner::setupCollision()
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

void LocalPlanner::planPath()
{    
    initializePlanner();
    const ros::Time &start_time = ros::Time::now();
    ros::Duration duration;
    while(true)
    {
        duration = ros::Time::now() - start_time;
        if(duration.toSec() * 1000 > m_timeout_ms)
        {            
            m_retry_it++;
            if(m_retry_it > m_num_retries - 1)
            {
                ROS_WARN_STREAM("Failed to Find Local Path After " << m_num_retries << " Attempts. Resetting");
                m_retry_it = 0;
                pubHealth(false);
            }
            return;
        }
        GraphNode current_node = m_frontier.top();
        while(checkForCollision(calcNodeTransform(current_node.child_point, current_node.heading)))
        {
            duration = ros::Time::now() - start_time;
            if(duration.toSec() * 1000 > m_timeout_ms)
            {
                m_retry_it++;
                if(m_retry_it > m_num_retries - 1)
                {
                    ROS_WARN_STREAM("Failed to Find Local Path After " << m_num_retries << " Attempts. Resetting");
                    m_retry_it = 0;
                    pubHealth(false);
                }
                return;
            }
            m_frontier.pop();
            closeNode(current_node);
            current_node = m_frontier.top();
        }
        if(checkForGoal(current_node))
        {
            m_retry_it = 0;
            pubHealth(true);
            reconstructTrajectory();
            break;
        }
        m_frontier.pop();
        closeNode(current_node);
        expandFrontier(current_node);
    }        
}

void LocalPlanner::initializePlanner()
{
    clearFrontier();
    setupCollision();
    calcPossibleYawRates();
}

void LocalPlanner::clearFrontier()
{
    m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost>();
    m_open_nodes.clear();    
    m_closed_nodes.clear();    
    m_nodes.clear();
    m_possible_yaw_rates.clear();
    m_node_id = 0;
    double roll, pitch, yaw;
    tf::Quaternion q;
    q.setW(m_pose->pose.pose.orientation.w);
    q.setX(m_pose->pose.pose.orientation.x);
    q.setY(m_pose->pose.pose.orientation.y);
    q.setZ(m_pose->pose.pose.orientation.z);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const double &vel = std::sqrt(std::pow(m_turtlebot_velocity.linear.x, 2) + std::pow(m_turtlebot_velocity.linear.y, 2));
    const GraphNode &node = GraphNode(Point<double>(m_pose->pose.pose.position.x, m_pose->pose.pose.position.y),
                                      Point<double>(m_pose->pose.pose.position.x, m_pose->pose.pose.position.y),
                                      m_node_id, m_node_id, yaw, 0, vel, 0.0, std::numeric_limits<double>::infinity());
    m_node_id++;
    m_frontier.push(node);
    openNode(node);
    m_nodes.insert(std::make_pair(m_node_id, node));
    m_node_id++;
}

void LocalPlanner::openNode(const GraphNode &node)
{
    m_frontier.push(node);
    m_open_nodes.push_back(node);    
}

void LocalPlanner::closeNode(const GraphNode &node)
{
    const std::vector<GraphNode>::iterator &it = std::find(m_open_nodes.begin(), m_open_nodes.end(), node);
    if(it != m_open_nodes.end())
    {
        m_closed_nodes.push_back(*it);
    }
    else
    {
        return;
    }
    m_open_nodes.erase(it);
}

const bool LocalPlanner::checkForGoal(const GraphNode &node)
{
    const double &time_step = m_time_step_ms / 1000;
    const double &dist_to_goal = calcDistanceBetween(node.child_point, Point<double>(m_goal_pose.x, m_goal_pose.y));
    const double &max_travel_dist = node.velocity * time_step + m_goal_pos_tolerance;
    const double &heading_diff = fabs(m_goal_pose.heading - node.heading);
    const double &velocity_diff = fabs(m_goal_pose.speed - node.velocity);
    if((dist_to_goal > max_travel_dist + m_goal_pos_tolerance) ||
       (heading_diff > m_max_yaw_rate * time_step + m_goal_heading_tolerance) ||
       (velocity_diff > m_goal_pose.max_accel * time_step + m_goal_speed_tolerance))
    {        
        return false;
    }
    m_frontier.pop();
    closeNode(node);
    GraphNode goal_node(Point<double>(m_goal_pose.x, m_goal_pose.y),
                        node.child_point,
                        m_node_id,
                        node.id,
                        m_goal_pose.heading,
                        0,
                        m_goal_pose.speed,
                        0, 0);
    m_frontier.push(goal_node);
    return true;
}

void LocalPlanner::expandFrontier(const GraphNode &current_node)
{
    const std::vector<GraphNode> &neighbors = getNeighbors(current_node);
    for(const auto &neighbor : neighbors)
    {
        const std::vector<GraphNode>::iterator &it_closed = std::find(m_closed_nodes.begin(), m_closed_nodes.end(), neighbor);
        if(it_closed != m_closed_nodes.end())
        {
           continue;
        }
        const std::vector<GraphNode>::iterator &it_open = std::find(m_open_nodes.begin(), m_open_nodes.end(), neighbor);
        if(it_open != m_open_nodes.end())
        {
            if(neighbor.cost >= (*it_open).cost)
            {
                continue;
            }
        }
        else
        {
            openNode(neighbor);
        }
    }
}

const std::vector<GraphNode> LocalPlanner::getNeighbors(const GraphNode &current_node)
{
    std::vector<GraphNode> neighbors;
    const Spline1d &spline = calcSpline(current_node.child_point, current_node.heading);
    const std::vector<double> &possible_velocities = calcPossibleVelocities(current_node);
    for(const auto &velocity : possible_velocities)
    {
        for(const auto &yaw_rate : m_possible_yaw_rates)
        {
            double r = velocity / yaw_rate;
            if(std::isnan(r) || std::isinf(r))
            {
                r = 0;
            }
            const double &x = current_node.child_point.x - r * sin(current_node.heading) + r * sin(current_node.heading + yaw_rate * m_time_step_ms / 1000);
            const double &y = current_node.child_point.y + r * cos(current_node.heading) - r * cos(current_node.heading + yaw_rate * m_time_step_ms / 1000);
            const double &heading = current_node.heading + yaw_rate * m_time_step_ms / 1000;
            const Point<double> new_pt(x, y);           
            const double &g = calcG(new_pt, current_node);
            const double &h = calcH(new_pt, current_node.child_point, heading, velocity, spline);
            GraphNode new_node(new_pt, current_node.child_point, m_node_id, current_node.id, heading, yaw_rate, velocity, g, g + h);
            neighbors.push_back(new_node);
            m_nodes.insert(std::make_pair(m_node_id, new_node));
            m_node_id++;
        }
    }    
    return neighbors;
}

const std::vector<double> LocalPlanner::calcPossibleVelocities(const GraphNode &current_node) const
{
    std::vector<double> possible_velocities;
    const double &current_velocity = current_node.velocity;
    const double &max_accel = m_goal_pose.max_accel;
    const double &max_velocity  = m_goal_pose.max_speed;
    const double &time_step = m_time_step_ms / 1000;
    const double &max_possible_vel = current_velocity + max_accel * time_step;
    const double &min_possible_vel = current_velocity - max_accel * time_step;
    const double &possible_vel_range = max_possible_vel - min_possible_vel;
    const int &num_possible_vels = possible_vel_range / m_velocity_res;
    for(int i = 0; i < num_possible_vels; i++)
    {
        double velocity = min_possible_vel + i * m_velocity_res;
        if(velocity > max_velocity)
        {
            velocity = max_velocity;
        }
        else if(velocity < 0)
        {
            velocity = 0;
        }
        possible_velocities.push_back(velocity);
    }
    return possible_velocities;
}

void LocalPlanner::calcPossibleYawRates()
{
    const int &num_possible_headings = 2 * m_max_yaw_rate / m_yaw_rate_res;
    for(int i = 0; i < num_possible_headings; i++)
    {        
        const double &yaw_rate = -m_max_yaw_rate + i * m_yaw_rate_res;
        m_possible_yaw_rates.push_back(yaw_rate);
    }    
}

const tf::Transform LocalPlanner::calcNodeTransform(const Point<double> &pt, const double &heading) const
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pt.x, pt.y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(heading));
    return transform;
}

const double LocalPlanner::calcG(const Point<double> &pt, const GraphNode &parent_node) const
{
    return parent_node.g + calcDistanceBetween(pt, parent_node.child_point);
}

const double LocalPlanner::calcH(const Point<double> &pt, const Point<double> &cur_pt, const double &heading, const double &velocity, const Spline1d &spline)
{
    const double &max_dist = calcDistanceBetween(Point<double>(m_pose->pose.pose.position.x, m_pose->pose.pose.position.y),
                                                 Point<double>(m_goal_pose.x, m_goal_pose.y));
    const double &dist_to_goal = calcDistanceBetween(pt, Point<double>(m_goal_pose.x, m_goal_pose.y));
    const double &dist_heuristic = dist_to_goal / max_dist * 100;
    const double &target_heading = calcTargetHeading(cur_pt, velocity, spline);
    const double &heading_diff = fabs(target_heading - heading);
    double heading_heuristic = heading_diff / M_PI * 250;
    double velocity_heuristic = 100 - velocity / m_goal_pose.max_speed * 100;
    if(checkVelH(pt, heading, velocity))
    {
        velocity_heuristic = fabs(velocity - m_goal_pose.speed) / (m_goal_pose.speed - m_goal_pose.max_speed) * 100;
    }
    return fabs(dist_heuristic + heading_heuristic + velocity_heuristic);
}

const Spline1d LocalPlanner::calcSpline(const Point<double> &pt, const double &heading) const
{
    const double &start_x = pt.x;
    const double &start_y = pt.y;
    const double &start_x_ = start_x + 0.001 * cos(heading);
    const double &start_y_ = start_y + 0.001 * sin(heading);
    const double &end_x = m_goal_pose.x;
    const double &end_y = m_goal_pose.y;
    const double &end_x_ = end_x - 0.001 * cos(m_goal_pose.heading);
    const double &end_y_ = end_y - 0.001 * sin(m_goal_pose.heading);
    Eigen::MatrixXd points(2, 4);
    points << start_x, start_x_, end_x_, end_x,
              start_y, start_y_, end_y_, end_y;
    const Spline1d &spline = Spline1dFitting::Interpolate(points, m_spline_order);
    return spline;
}

const double LocalPlanner::calcTargetHeading(const Point<double> &pt, const double &velocity, const Spline1d &spline)
{
    const double &dx = pt.x + velocity * m_time_step_ms / 1000;
    const double &dy = pt.y + velocity * m_time_step_ms / 1000;
    const double &dist = sqrt(pow(dx, 2) + pow(dy, 2));
    const double &spline_res = 0.01;
    double spline_value = 0;
    double spline_dist = 0;
    double x, y;
    double prev_x = pt.x;
    double prev_y  = pt.y;
    while(spline_dist < dist)
    {
        const Eigen::MatrixXd &spline_pt = spline(spline_value);
        x = spline_pt(0);
        y = spline_pt(1);
        const double &dp = sqrt(pow(x - prev_x, 2) + pow(y - prev_y, 2));
        spline_dist += dp;
        spline_value += spline_res;
    }
    return atan2(y - pt.y, x - pt.x);
}

const bool LocalPlanner::checkVelH(const Point<double> &pt, const double &heading, const double &velocity)
{
    double next_vel = velocity + m_goal_pose.max_accel * m_time_step_ms / 1000;
    if(next_vel > m_goal_pose.max_speed)
    {
        next_vel = m_goal_pose.max_speed;
    }
    const double &average_vel = (next_vel + velocity) / 2;
    const double &x = pt.x + average_vel * m_time_step_ms / 1000 * cos(heading);
    const double &y = pt.y + average_vel * m_time_step_ms / 1000 * sin(heading);
    const double &dx = x - m_goal_pose.x;
    const double &dy = y - m_goal_pose.y;
    const double &dist_to_goal = sqrt(pow(dx, 2) + pow(dy, 2));
    const double &speed = velocity;
    const double &max_accel = m_goal_pose.max_accel;
    const double &time_to_decel = (speed - m_goal_pose.speed) / max_accel;
    const double &dist_to_decel = speed * time_to_decel - m_goal_pose.max_accel * pow(time_to_decel, 2) / 2;
    return (dist_to_goal <= dist_to_decel);
}

const double LocalPlanner::calcDistanceBetween(const Point<double> &lhs, const Point<double> &rhs) const
{
    const double &dx = lhs.x - rhs.x;
    const double &dy = lhs.y - rhs.y;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

const bool LocalPlanner::checkForCollision(const tf::Transform &transform) const
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

const bool LocalPlanner::checkPointForCollision(const tf::Point &pt) const
{
    return int(m_map->data[calcGridLocation(Point<double>(pt.getX(), pt.getY()))]) == 100;
}

const int &LocalPlanner::calcGridLocation(const Point<double> &pt) const
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

void LocalPlanner::reconstructTrajectory()
{
    std::vector<GraphNode> reverse_trajectory;
    GraphNode current_node = m_frontier.top();
    while(current_node.parent_id != 0)
    {
        reverse_trajectory.push_back(current_node);
        current_node = m_nodes[current_node.parent_id];
    }
    pubTrajectory(reverseTrajectory(reverse_trajectory));
}

const std::vector<GraphNode> LocalPlanner::reverseTrajectory(const std::vector<GraphNode> &reverse_traj)
{
    std::vector<GraphNode> traj;
    for(int traj_id = reverse_traj.size() - 1; traj_id >= 0; traj_id--)
    {
        traj.push_back(reverse_traj[traj_id]);
    }
    return traj;
}

void LocalPlanner::pubTrajectory(const std::vector<GraphNode> &traj)
{
    turtlebot_msgs::Trajectory trajectory;
    nav_msgs::Path path;
    trajectory.header.stamp = ros::Time::now();
    path.header.stamp = m_goal_pose.header.stamp;
    trajectory.max_accel = m_goal_pose.max_accel;
    trajectory.max_speed = m_goal_pose.max_speed;
    path.header.frame_id = "/map";
    for(int traj_it = 0; traj_it < traj.size(); traj_it++)
    {
        const double &x = traj[traj_it].child_point.x;
        const double &y = traj[traj_it].child_point.y;
        const double &heading = traj[traj_it].heading;
        const double &yaw_rate = traj[traj_it].yaw_rate;
        const double &velocity = traj[traj_it].velocity;
        trajectory.x_values.push_back(x);
        trajectory.y_values.push_back(y);
        trajectory.speeds.push_back(velocity);
        trajectory.headings.push_back(heading);
        trajectory.yaw_rates.push_back(yaw_rate);
        trajectory.durations.push_back(m_time_step_ms / 1000);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }
    m_trajectory_pub.publish(trajectory);
    m_path_pub.publish(path);
}

void LocalPlanner::pubHealth(const bool &health)
{
    std_msgs::Bool health_;
    health_.data = health;
    m_health_pub.publish(health_);
}

void LocalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_costmap)
    {
        m_have_costmap = true;
        m_local_costmap_height = msg->info.height;
        m_local_costmap_width = msg->info.width;
        m_local_costmap_res = msg->info.resolution;
    }
    m_map = msg;
}

void LocalPlanner::goalPoseCallback(const turtlebot_msgs::GoalPose::ConstPtr &msg)
{
    if(m_have_costmap && m_have_pose && m_have_odom)
    {
        have_goal = true;
        m_goal_pose = *msg;
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "/map";
        goal_pose.pose.position.x = msg->x;
        goal_pose.pose.position.y = msg->y;
        goal_pose.pose.position.z = 0;
        goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg->heading);
        m_goal_pub.publish(goal_pose);
        planPath();
    }
}

void LocalPlanner::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose)
    {
        m_have_pose = true;
    }
    m_pose = msg;
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
    }
    m_turtlebot_velocity = msg->twist.twist;
}
}


