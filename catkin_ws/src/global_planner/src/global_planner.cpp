#include <global_planner.hpp>

namespace Turtlebot
{

GlobalPlanner::GlobalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &GlobalPlanner::costmapCallback, this);
    m_goal_pose_sub = nh.subscribe<turtlebot_msgs::GoalPose>("/global_goal_pose", 10, &GlobalPlanner::goalPoseCallback, this);
    m_rviz_goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GlobalPlanner::rvizGoalPoseCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose", 10, &GlobalPlanner::poseCallback, this);
    m_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 10);
    m_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/global_goal", 100);
    getParams(pnh);
}

GlobalPlanner::~GlobalPlanner(){}

void GlobalPlanner::getParams(ros::NodeHandle &pnh)
{
      pnh.getParam("collision_buffer_distance", m_collision_buffer_distance);
      pnh.getParam("search_res", m_search_res);
      pnh.getParam("angular_search_res", m_angular_search_res);
      pnh.getParam("spline_order", m_spline_order);
      pnh.getParam("goal_pos_tolerance", m_goal_pos_tolerance);
      pnh.getParam("timeout_ms", m_timeout_ms);
      pnh.getParam("turtlebot_radius", m_radius);
      pnh.getParam("angular_col_res", m_angular_col_res);
      pnh.getParam("radial_col_res", m_radial_col_res);
}

void GlobalPlanner::setupCollision()
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

void GlobalPlanner::planPath()
{    
    initializePlanner();
    const ros::Time &start_time = ros::Time::now();
    ros::Duration duration;
    while(true)
    {
        duration = ros::Time::now() - start_time;
        if(duration.toSec() * 1000 > m_timeout_ms)
        {            
            ROS_WARN_STREAM("Failed to Find Path After " << m_timeout_ms / 1000 << " Seconds");
            return;
        }
        GraphNode current_node = m_frontier.top();
        while(checkForCollision(calcNodeTransform(current_node.child_point)))
        {
            duration = ros::Time::now() - start_time;
            if(duration.toSec() * 1000 > m_timeout_ms)
            {
                ROS_WARN_STREAM("Failed to Find Path After " << m_timeout_ms / 1000 << " Seconds");
                return;
            }
            m_frontier.pop();
            closeNode(current_node);
            current_node = m_frontier.top();
        }
        if(checkForGoal(current_node))
        {
            reconstructTrajectory();
            break;
        }
        m_frontier.pop();
        closeNode(current_node);
        expandFrontier(current_node);
    }        
}

void GlobalPlanner::initializePlanner()
{
    clearFrontier();
    setupCollision();
}

void GlobalPlanner::clearFrontier()
{
    m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost>();
    m_open_nodes.clear();    
    m_closed_nodes.clear();    
    m_nodes.clear();
    m_node_id = 0;
    const GraphNode &node = GraphNode(Point<double>(m_pose->pose.pose.position.x, m_pose->pose.pose.position.y),
                                      Point<double>(m_pose->pose.pose.position.x, m_pose->pose.pose.position.y),
                                      m_node_id, m_node_id, 0.0, std::numeric_limits<double>::infinity());
    m_node_id++;
    m_frontier.push(node);
    openNode(node);
    m_nodes.insert(std::make_pair(m_node_id, node));
    m_node_id++;
}

void GlobalPlanner::openNode(const GraphNode &node)
{
    m_frontier.push(node);
    m_open_nodes.push_back(node);    
}

void GlobalPlanner::closeNode(const GraphNode &node)
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

const bool GlobalPlanner::checkForGoal(const GraphNode &node)
{
    const double &dist_to_goal = calcDistanceBetween(node.child_point, Point<double>(m_goal_pose.x, m_goal_pose.y));   
    if(dist_to_goal > m_goal_pos_tolerance)
    {        
        return false;
    }
    m_frontier.pop();
    closeNode(node);
    GraphNode goal_node(Point<double>(m_goal_pose.x, m_goal_pose.y),
                        node.child_point,
                        m_node_id,
                        node.id,
                        0, 0);
    m_frontier.push(goal_node);
    return true;
}

void GlobalPlanner::expandFrontier(const GraphNode &current_node)
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

const std::vector<GraphNode> GlobalPlanner::getNeighbors(const GraphNode &current_node)
{
    std::vector<GraphNode> neighbors;
    for(int th = 0; th < 2 * M_PI / m_angular_search_res; th++)
    {
        const double &x = current_node.child_point.x + m_search_res * cos(th);
        const double &y = current_node.child_point.y + m_search_res * sin(th);
        const Point<double> pt(x, y);
        const double &g = calcG(pt, current_node);
        const double &h = calcH(pt);
        GraphNode new_node(pt, current_node.child_point, m_node_id, current_node.id, g, g + h);
        neighbors.push_back(new_node);
        m_nodes.insert(std::make_pair(m_node_id, new_node));
        m_node_id++;
    }
    return neighbors;
}

const tf::Transform GlobalPlanner::calcNodeTransform(const Point<double> &pt) const
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pt.x, pt.y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(0));
    return transform;
}

const double GlobalPlanner::calcG(const Point<double> &pt, const GraphNode &parent_node) const
{
    return parent_node.g + calcDistanceBetween(pt, parent_node.child_point);
}

const double GlobalPlanner::calcH(const Point<double> &pt)
{
    const double &max_dist = calcDistanceBetween(Point<double>(m_pose->pose.pose.position.x, m_pose->pose.pose.position.y),
                                                 Point<double>(m_goal_pose.x, m_goal_pose.y));
    const double &dist_to_goal = calcDistanceBetween(pt, Point<double>(m_goal_pose.x, m_goal_pose.y));
    return dist_to_goal / max_dist * 100;
}

const double GlobalPlanner::calcDistanceBetween(const Point<double> &lhs, const Point<double> &rhs) const
{
    const double &dx = lhs.x - rhs.x;
    const double &dy = lhs.y - rhs.y;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

const bool GlobalPlanner::checkForCollision(const tf::Transform &transform) const
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

const bool GlobalPlanner::checkPointForCollision(const tf::Point &pt) const
{
    return int(m_map->data[calcGridLocation(Point<double>(pt.getX(), pt.getY()))]) == 100;
}

const int &GlobalPlanner::calcGridLocation(const Point<double> &pt) const
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

void GlobalPlanner::reconstructTrajectory()
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

const std::vector<GraphNode> GlobalPlanner::reverseTrajectory(const std::vector<GraphNode> &reverse_traj)
{
    std::vector<GraphNode> traj;
    for(int traj_id = reverse_traj.size() - 1; traj_id >= 0; traj_id--)
    {
        traj.push_back(reverse_traj[traj_id]);
    }
    return traj;
}

void GlobalPlanner::pubTrajectory(const std::vector<GraphNode> &traj)
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
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }
    m_path_pub.publish(path);
}

void GlobalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_costmap)
    {
        m_have_costmap = true;
        m_global_costmap_height = msg->info.height;
        m_global_costmap_width = msg->info.width;
        m_global_costmap_res = msg->info.resolution;
    }
    m_map = msg;
}

void GlobalPlanner::goalPoseCallback(const turtlebot_msgs::GoalPose::ConstPtr &msg)
{
    if(m_have_costmap && m_have_pose)
    {
        have_goal = true;
        m_goal_pose = *msg;
        planPath();
    }
}

void GlobalPlanner::rvizGoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(m_have_costmap && m_have_pose)
    {
        have_goal = true;
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.orientation, q);
        m_goal_pose.heading = tf::getYaw(q);
        m_goal_pose.x = msg->pose.position.x;
        m_goal_pose.y = msg->pose.position.y;
        ROS_INFO_STREAM("Global Goal Pose Updated Manually");
        planPath();
    }
}

void GlobalPlanner::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose)
    {
        m_have_pose = true;
    }
    m_pose = msg;
}

}


